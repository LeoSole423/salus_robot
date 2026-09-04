#!/usr/bin/env python3
"""Static contract tests for the real MVP deployment assets."""

from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).parents[1]
UNIT = ROOT / "deploy/systemd/salus-robot-real.service"
ENV = ROOT / "deploy/systemd/salus-robot-real.env.example"
START = ROOT / "tools/start_real_runtime.sh"
READY = ROOT / "tools/check_real_mvp_readiness.sh"
RUNBOOK = ROOT / "docs/runbooks/real-mvp.md"


class DeploymentContractTests(unittest.TestCase):
    def test_unit_has_fail_closed_order_and_explicit_devices(self) -> None:
        text = UNIT.read_text(encoding="utf-8")
        self.assertIn("ExecStartPre=/opt/salus_robot/tools/check_real_mvp_authority.py", text)
        self.assertIn("ExecStart=/opt/salus_robot/tools/start_real_runtime.sh", text)
        self.assertIn("ExecStartPost=/opt/salus_robot/tools/check_real_mvp_readiness.sh", text)
        self.assertIn("ConditionPathExists=/dev/ttyACM0", text)
        self.assertIn("ConditionPathExists=/dev/ttyUSB0", text)
        self.assertIn("SupplementaryGroups=dialout docker", text)
        self.assertIn("KillMode=control-group", text)
        self.assertIn("KillSignal=SIGINT", text)
        self.assertNotIn("--privileged", text)
        self.assertNotIn("salus-real-global-v2-wifi.service", text)

    def test_start_wrapper_reuses_runtime_and_maps_private_config(self) -> None:
        text = START.read_text(encoding="utf-8")
        self.assertIn("real_runtime_exec.sh", text)
        self.assertIn("--device /dev/ttyACM0", text)
        self.assertIn("--device /dev/ttyUSB0", text)
        self.assertIn("ros2 launch salus_bringup real_mvp.launch.py", text)
        self.assertIn("ntrip_config_path:=${ntrip_config_container}", text)
        self.assertIn("serial_port:=${SALUS_SERIAL_PORT}", text)
        self.assertNotIn("password", text.lower())
        self.assertNotIn("--privileged", text)

    def test_readiness_is_minimal_and_causal(self) -> None:
        text = READY.read_text(encoding="utf-8")
        self.assertIn("real_runtime_exec.sh", text)
        self.assertIn("/navigation_startup/diagnostics", text)
        self.assertIn("ACTIVE: ALL_NAV2_NODES_ACTIVE", text)
        self.assertNotIn("sleep ", text)

    def test_external_config_and_runbook_are_explicit(self) -> None:
        env = ENV.read_text(encoding="utf-8")
        runbook = RUNBOOK.read_text(encoding="utf-8")
        self.assertIn("src/salus_hardware/config/rtk_sources.local.yaml", env)
        for item in (
            "prepare_real_runtime.sh",
            "real_runtime_exec.sh",
            "salus-real-global-v2-wifi.service",
            "/dev/ttyACM0",
            "/dev/ttyUSB0",
            "0600",
            "rollback",
        ):
            self.assertIn(item, runbook)
        self.assertNotIn("password", env.lower())

    def test_authority_checker_help_is_available_without_systemd(self) -> None:
        result = subprocess.run(
            ["python3", str(ROOT / "tools/check_real_mvp_authority.py"), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
