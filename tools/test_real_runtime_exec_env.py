"""Focused contract tests for the real-runtime DDS environment."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = SOURCE_ROOT / "tools" / "real_runtime_exec.sh"


class RealRuntimeExecEnvironmentTest(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")

    def _evaluate_assignment(self, variable: str, env: dict[str, str]) -> str:
        prefix = f"{variable}="
        assignment = next(
            line for line in self.source.splitlines() if line.startswith(prefix)
        )
        result = subprocess.run(
            ["bash", "-c", f"set -u\n{assignment}\nprintf '%s' \"${{{variable}}}\""],
            env=env,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return result.stdout

    def test_real_runtime_dds_defaults_and_docker_propagation(self) -> None:
        env = os.environ.copy()
        env.pop("RMW_IMPLEMENTATION", None)
        env.pop("ROS_DOMAIN_ID", None)

        self.assertEqual(
            self._evaluate_assignment("runtime_rmw_implementation", env),
            "rmw_cyclonedds_cpp",
        )
        self.assertEqual(self._evaluate_assignment("runtime_ros_domain_id", env), "0")
        self.assertIn(
            '-e "RMW_IMPLEMENTATION=${runtime_rmw_implementation}"', self.source
        )
        self.assertIn('-e "ROS_DOMAIN_ID=${runtime_ros_domain_id}"', self.source)
        self.assertIn('-e "ROS_LOCALHOST_ONLY=0"', self.source)

    def test_operator_ros_domain_id_is_preserved(self) -> None:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "42"
        self.assertEqual(self._evaluate_assignment("runtime_ros_domain_id", env), "42")


if __name__ == "__main__":
    unittest.main()
