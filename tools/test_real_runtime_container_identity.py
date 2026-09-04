#!/usr/bin/env python3
"""Causal tests for named and ephemeral prepared-runtime containers."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]


class RealRuntimeContainerIdentityTests(unittest.TestCase):
    def _fixture(self) -> tuple[Path, dict[str, str], Path, Path, Path, Path]:
        temp_dir = Path(tempfile.mkdtemp())
        tools_dir = temp_dir / "tools"
        tools_dir.mkdir()
        for name in ("real_runtime_common.sh", "real_runtime_exec.sh"):
            shutil.copy2(SOURCE_ROOT / "tools" / name, tools_dir / name)
        for name in ("Dockerfile.real", "entrypoint.sh", "dependencies.repos"):
            shutil.copy2(SOURCE_ROOT / name, temp_dir / name)

        subprocess.run(["git", "init", "-q"], cwd=temp_dir, check=True)
        subprocess.run(["git", "add", "."], cwd=temp_dir, check=True)
        subprocess.run(
            [
                "git",
                "-c",
                "user.name=test",
                "-c",
                "user.email=test@example.invalid",
                "commit",
                "-qm",
                "fixture",
            ],
            cwd=temp_dir,
            check=True,
        )

        cache_dir = temp_dir.parent / f"{temp_dir.name}-cache"
        workspace_dir = temp_dir.parent / f"{temp_dir.name}-workspace"
        for child in (workspace_dir / "build", workspace_dir / "install", workspace_dir / "log"):
            child.mkdir(parents=True)
        source_sha = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=temp_dir,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
        hashes = subprocess.run(
            [
                "bash",
                "-c",
                'source "$1"; salus_recipe_hash "$(id -u)" "$(id -g)"',
                "bash",
                str(tools_dir / "real_runtime_common.sh"),
            ],
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
        deps_hash = subprocess.run(
            ["sha256sum", str(temp_dir / "dependencies.repos")],
            check=True,
            text=True,
            capture_output=True,
        ).stdout.split()[0]
        state = (
            f"STATE_SOURCE_SHA={source_sha}\n"
            "STATE_IMAGE_ID=sha256:fixture\n"
            f"STATE_RECIPE_HASH={hashes}\n"
            f"STATE_DEPS_HASH={deps_hash}\n"
        )
        (workspace_dir / "state.env").write_text(state, encoding="utf-8")
        prepared = (
            f"SOURCE_SHA={source_sha}\n"
            "IMAGE_ID=sha256:fixture\n"
            f"RECIPE_HASH={hashes}\n"
            f"DEPS_HASH={deps_hash}\n"
            "WORKSPACE_KEY=fixture\n"
            f"WORKSPACE_DIR={workspace_dir}\n"
            f"CACHE_DIR={cache_dir}\n"
            f"DEPS_CACHE_DIR={temp_dir}\n"
        )
        (cache_dir).mkdir()
        (cache_dir / "prepared.env").write_text(prepared, encoding="utf-8")

        bin_dir = temp_dir.parent / f"{temp_dir.name}-bin"
        bin_dir.mkdir()
        docker_log = temp_dir.parent / f"{temp_dir.name}-docker-args"
        docker = bin_dir / "docker"
        docker.write_text(
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "printf '%s\\n' \"$@\" >> \"${SALUS_DOCKER_LOG}\"\n"
            "if [[ \"${1:-}\" == image && \"${2:-}\" == inspect ]]; then\n"
            "  printf 'sha256:fixture\\n'\n"
            "fi\n",
            encoding="utf-8",
        )
        docker.chmod(0o755)
        env = os.environ | {
            "HOME": str(temp_dir),
            "PATH": f"{bin_dir}:{os.environ['PATH']}",
            "SALUS_REAL_CACHE_DIR": str(cache_dir),
            "SALUS_DOCKER_LOG": str(docker_log),
        }
        return temp_dir, env, docker_log, workspace_dir, cache_dir, bin_dir

    def _run_runtime(self, script: Path, env: dict[str, str], name: str | None) -> list[str]:
        command = [str(script)]
        if name is not None:
            command.extend(["--container-name", name])
        command.extend(["--", "true"])
        result = subprocess.run(command, env=env, check=False, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        return (Path(env["SALUS_DOCKER_LOG"]).read_text(encoding="utf-8")).splitlines()

    def test_only_explicit_main_runtime_gets_deterministic_name(self) -> None:
        temp_dir, env, docker_log, workspace_dir, cache_dir, bin_dir = self._fixture()
        try:
            ephemeral_args = self._run_runtime(temp_dir / "tools/real_runtime_exec.sh", env, None)
            docker_log.unlink()
            named_args = self._run_runtime(
                temp_dir / "tools/real_runtime_exec.sh", env, "salus-robot-real-runtime"
            )
        finally:
            shutil.rmtree(temp_dir)
            shutil.rmtree(workspace_dir)
            shutil.rmtree(cache_dir)
            shutil.rmtree(bin_dir)
            docker_log.unlink(missing_ok=True)

        self.assertNotIn("--name", ephemeral_args)
        self.assertIn("--name", named_args)
        name_index = named_args.index("--name")
        self.assertEqual(named_args[name_index + 1], "salus-robot-real-runtime")


if __name__ == "__main__":
    unittest.main()
