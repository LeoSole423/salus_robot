#!/usr/bin/env python3
"""Pure regression tests for the real MVP readiness diagnostic contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
PROBE_PATH = ROOT / "tools" / "real_mvp_readiness_probe.py"
SPEC = importlib.util.spec_from_file_location("real_mvp_readiness_probe", PROBE_PATH)
assert SPEC is not None and SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class RealMvpReadinessProbeTests(unittest.TestCase):
    def test_active_status_is_required_exactly(self) -> None:
        self.assertTrue(
            probe.has_active_startup(
                [
                    SimpleNamespace(name="other", message=probe.ACTIVE_MESSAGE),
                    SimpleNamespace(
                        name="navigation_startup", message=probe.ACTIVE_MESSAGE
                    ),
                ]
            )
        )

    def test_non_active_statuses_do_not_pass(self) -> None:
        self.assertFalse(
            probe.has_active_startup(
                [
                    SimpleNamespace(name="other", message=probe.ACTIVE_MESSAGE),
                    SimpleNamespace(
                        name="navigation_startup", message="WAITING: TF_MISSING"
                    ),
                ]
            )
        )

    def test_runtime_probe_waits_for_diagnostics_callback(self) -> None:
        class FakeNode:
            def __init__(self) -> None:
                self.callback = None
                self.destroyed = False

            def create_subscription(self, _type, topic, callback, _depth) -> None:
                self.topic = topic
                self.callback = callback

            def destroy_node(self) -> None:
                self.destroyed = True

        node = FakeNode()
        fake_rclpy = SimpleNamespace(
            init=lambda args: None,
            create_node=lambda _name: node,
            ok=lambda: True,
            shutdown=lambda: None,
        )

        def spin_once(fake_node, timeout_sec: float) -> None:
            self.assertGreater(timeout_sec, 0.0)
            self.assertIsNotNone(fake_node.callback)
            fake_node.callback(
                SimpleNamespace(
                    status=[
                        SimpleNamespace(
                            name="navigation_startup", message=probe.ACTIVE_MESSAGE
                        )
                    ]
                )
            )

        fake_rclpy.spin_once = spin_once
        diagnostic_msgs = SimpleNamespace(msg=SimpleNamespace(DiagnosticArray=object))
        with patch.dict(
            sys.modules,
            {
                "rclpy": fake_rclpy,
                "diagnostic_msgs": diagnostic_msgs,
                "diagnostic_msgs.msg": diagnostic_msgs.msg,
            },
        ):
            self.assertTrue(probe.wait_for_active_startup(1.0))

        self.assertEqual(node.topic, "/navigation_startup/diagnostics")
        self.assertTrue(node.destroyed)

    def test_shell_probe_waits_with_rclpy_instead_of_requiring_topic_type_first(self) -> None:
        shell = (ROOT / "tools" / "check_real_mvp_readiness.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("real_mvp_readiness_probe.py", shell)
        self.assertIn("real_runtime_exec.sh", shell)
        self.assertNotIn("ros2 topic echo", shell)
        self.assertNotIn("sleep ", shell)


if __name__ == "__main__":
    unittest.main()
