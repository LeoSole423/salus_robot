"""Control-flow tests for the persistent real-runtime cache."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]


class RealRuntimeCacheTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory(prefix="salus-real-cache-test-")
        self.root = Path(self.tempdir.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        (self.repo / "tools").mkdir()
        (self.repo / "src").mkdir()
        for name in (
            "real_runtime_common.sh",
            "prepare_real_runtime.sh",
            "real_runtime_exec.sh",
            "build_real_image.sh",
        ):
            shutil.copy2(SOURCE_ROOT / "tools" / name, self.repo / "tools" / name)
        shutil.copy2(SOURCE_ROOT / "Dockerfile.real", self.repo / "Dockerfile.real")
        shutil.copy2(SOURCE_ROOT / "entrypoint.sh", self.repo / "entrypoint.sh")
        shutil.copy2(SOURCE_ROOT / "dependencies.repos", self.repo / "dependencies.repos")

        self._git("init", "-q")
        self._git("config", "user.email", "test@example.invalid")
        self._git("config", "user.name", "Runtime Cache Test")
        (self.repo / "marker.txt").write_text("base\n", encoding="utf-8")
        (self.repo / "src" / "marker.txt").write_text("source\n", encoding="utf-8")
        self._git("add", ".")
        self._git("commit", "-qm", "base")
        self.initial_sha = self._git_output("rev-parse", "HEAD")

        self.fake_bin = self.root / "bin"
        self.fake_bin.mkdir()
        self.cache = self.root / "cache"
        self.docker_log = self.root / "docker.log"
        self.docker_state = self.root / "docker.state"
        self.docker_state.write_text("sha256:base\n", encoding="utf-8")
        self._write_fake_docker()

        self.env = os.environ.copy()
        self.env.update(
            {
                "PATH": f"{self.fake_bin}:{self.env['PATH']}",
                "SALUS_REAL_CACHE_DIR": str(self.cache),
                "SALUS_REAL_IMAGE": "salus-robot:humble-real",
                "FAKE_DOCKER_LOG": str(self.docker_log),
                "FAKE_DOCKER_STATE": str(self.docker_state),
            }
        )

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def _git(self, *args: str) -> None:
        subprocess.run(
            ["git", *args], cwd=self.repo, check=True, stdout=subprocess.PIPE
        )

    def _git_output(self, *args: str) -> str:
        return subprocess.check_output(["git", *args], cwd=self.repo, text=True).strip()

    def _commit_marker(self, value: str) -> str:
        (self.repo / "marker.txt").write_text(f"{value}\n", encoding="utf-8")
        self._git("add", "marker.txt")
        self._git("commit", "-qm", value)
        return self._git_output("rev-parse", "HEAD")

    def _run(self, script: str, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(self.repo / "tools" / script), *args],
            cwd=self.repo,
            env=self.env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def _prepare_initial(self) -> subprocess.CompletedProcess[str]:
        return self._run("prepare_real_runtime.sh", "--adopt-validated-image")

    def _log(self) -> list[str]:
        if not self.docker_log.exists():
            return []
        return self.docker_log.read_text(encoding="utf-8").splitlines()

    def _write_fake_docker(self) -> None:
        fake = self.fake_bin / "docker"
        fake.write_text(
            r'''#!/usr/bin/env python3
import os
from pathlib import Path
import sys

log = Path(os.environ["FAKE_DOCKER_LOG"])
state = Path(os.environ["FAKE_DOCKER_STATE"])
args = sys.argv[1:]

def record(value):
    with log.open("a", encoding="utf-8") as stream:
        stream.write(value + "\n")

def image_id():
    return state.read_text(encoding="utf-8").strip()

if args[:2] == ["image", "inspect"]:
    record("inspect")
    fmt = args[args.index("--format") + 1] if "--format" in args else ""
    if "Architecture" in fmt:
        print("amd64")
    elif "Created" in fmt:
        print("2026-01-01T00:00:00Z")
    else:
        print(image_id())
    raise SystemExit(0)

if args and args[0] == "build":
    next_id = sum(1 for line in log.read_text(encoding="utf-8").splitlines()
                  if line == "build") + 1 if log.exists() else 1
    state.write_text(f"sha256:built{next_id}\n", encoding="utf-8")
    record("build")
    raise SystemExit(0)

if args and args[0] == "run":
    network = args[args.index("--network") + 1] if "--network" in args else "default"
    record(f"run network={network}")
    mounts = {}
    index = 0
    while index < len(args):
        if args[index] in ("-v", "--volume"):
            host, destination, *_ = args[index + 1].split(":", 2)
            mounts[destination] = Path(host)
            index += 2
        else:
            index += 1

    if network == "bridge":
        output = mounts["/output"]
        for package in ("rslidar_sdk", "rslidar_msg"):
            (output / "src" / package / ".git").mkdir(parents=True)
        (output / "src" / ".salus_complete").write_text("ok\n", encoding="utf-8")
        (output / ".salus_complete").write_text("ok\n", encoding="utf-8")
    elif network == "none":
        mounts["/ros2_ws/build"].mkdir(parents=True, exist_ok=True)
        mounts["/ros2_ws/install"].mkdir(parents=True, exist_ok=True)
        mounts["/ros2_ws/log"].mkdir(parents=True, exist_ok=True)
        (mounts["/ros2_ws/build"] / "fake-colcon.marker").write_text("built\n", encoding="utf-8")
    raise SystemExit(0)

record("unexpected " + " ".join(args))
raise SystemExit(2)
''',
            encoding="utf-8",
        )
        fake.chmod(fake.stat().st_mode | stat.S_IXUSR)

    def test_t1_identical_preparation_reuses_everything(self) -> None:
        first = self._prepare_initial()
        self.assertEqual(first.returncode, 0, first.stderr)
        before = len(self._log())
        second = self._run("prepare_real_runtime.sh")
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertIn("IMAGE_ACTION=reused", second.stdout)
        self.assertIn("DEPS_ACTION=reused", second.stdout)
        self.assertIn("WORKSPACE_ACTION=reused", second.stdout)
        self.assertEqual(
            [line for line in self._log() if line != "inspect"],
            [line for line in self._log()[:before] if line != "inspect"],
        )

    def test_t2_source_fast_forward_is_incremental_and_reuses_image(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        self._commit_marker("fast-forward")
        result = self._run("prepare_real_runtime.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=reused", result.stdout)
        self.assertIn("DEPS_ACTION=reused", result.stdout)
        self.assertIn("WORKSPACE_ACTION=incremental_build", result.stdout)
        self.assertEqual(sum(line == "build" for line in self._log()), 0)
        self.assertEqual(sum("network=bridge" in line for line in self._log()), 1)
        self.assertEqual(sum("network=none" in line for line in self._log()), 2)

    def test_t3_recipe_change_rebuilds_image_and_uses_new_workspace(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        old_workspace = self._prepared_value("WORKSPACE_KEY")
        with (self.repo / "Dockerfile.real").open("a", encoding="utf-8") as stream:
            stream.write("# recipe test change\n")
        self._git("add", "Dockerfile.real")
        self._git("commit", "-qm", "recipe change")
        result = self._run("prepare_real_runtime.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=rebuilt", result.stdout)
        self.assertNotEqual(old_workspace, self._prepared_value("WORKSPACE_KEY"))
        self.assertEqual(sum(line == "build" for line in self._log()), 1)

    def test_t4_dependency_change_imports_without_image_rebuild(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        old_workspace = self._prepared_value("WORKSPACE_KEY")
        with (self.repo / "dependencies.repos").open("a", encoding="utf-8") as stream:
            stream.write("\n# dependency manifest change\n")
        self._git("add", "dependencies.repos")
        self._git("commit", "-qm", "dependency change")
        result = self._run("prepare_real_runtime.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=reused", result.stdout)
        self.assertIn("DEPS_ACTION=imported", result.stdout)
        self.assertNotEqual(old_workspace, self._prepared_value("WORKSPACE_KEY"))
        self.assertEqual(sum(line == "build" for line in self._log()), 0)
        self.assertEqual(sum("network=bridge" in line for line in self._log()), 2)

    def test_t5_divergent_head_gets_clean_workspace_without_image_rebuild(self) -> None:
        prepared_sha = self._commit_marker("prepared")
        self.assertEqual(self._run("prepare_real_runtime.sh", "--adopt-validated-image").returncode, 0)
        self._git("checkout", "-q", "--detach", self.initial_sha)
        self._commit_marker("divergent")
        result = self._run("prepare_real_runtime.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=reused", result.stdout)
        self.assertIn("DEPS_ACTION=reused", result.stdout)
        self.assertIn("WORKSPACE_ACTION=clean_build", result.stdout)
        self.assertEqual(sum(line == "build" for line in self._log()), 0)
        self.assertEqual(sum("network=none" in line for line in self._log()), 2)
        self.assertNotEqual(prepared_sha, self._git_output("rev-parse", "HEAD"))

    def test_t6_dirty_tree_fails_before_mutating_reusable_state(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        prepared = (self.cache / "prepared.env").read_text(encoding="utf-8")
        before = len(self._log())
        (self.repo / "untracked.txt").write_text("dirty\n", encoding="utf-8")
        result = self._run("prepare_real_runtime.sh")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("working tree is dirty", result.stderr)
        self.assertEqual(prepared, (self.cache / "prepared.env").read_text(encoding="utf-8"))
        self.assertEqual(before, len(self._log()))

    def test_t7_exec_rejects_stale_source_before_docker(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        self._commit_marker("stale")
        before = len(self._log())
        result = self._run("real_runtime_exec.sh", "--", "ros2", "pkg", "prefix", "salus_description")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("runtime preparation is stale (source)", result.stderr)
        self.assertEqual(
            [line for line in self._log() if not line.startswith("inspect")],
            [line for line in self._log()[:before] if not line.startswith("inspect")],
        )

    def test_t8_force_flags_have_separate_effects(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        workspace_before = self._prepared_value("WORKSPACE_KEY")
        result = self._run("prepare_real_runtime.sh", "--force-workspace-rebuild")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=reused", result.stdout)
        self.assertIn("WORKSPACE_ACTION=clean_build", result.stdout)
        self.assertEqual(sum(line == "build" for line in self._log()), 0)
        self.assertEqual(workspace_before, self._prepared_value("WORKSPACE_KEY"))

        result = self._run("prepare_real_runtime.sh", "--force-image-rebuild")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMAGE_ACTION=rebuilt", result.stdout)
        self.assertEqual(sum(line == "build" for line in self._log()), 1)
        self.assertNotEqual(workspace_before, self._prepared_value("WORKSPACE_KEY"))

    def test_exec_uses_prepared_image_id_and_host_network(self) -> None:
        self.assertEqual(self._prepare_initial().returncode, 0)
        result = self._run("real_runtime_exec.sh", "--", "ros2", "pkg", "prefix", "salus_description")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(any("network=host" in line for line in self._log()))

    def _prepared_value(self, name: str) -> str:
        values = {}
        for line in (self.cache / "prepared.env").read_text(encoding="utf-8").splitlines():
            key, value = line.split("=", 1)
            values[key] = value
        return values[name]


if __name__ == "__main__":
    unittest.main()
