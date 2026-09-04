#!/usr/bin/env python3
"""Unit tests for the read-only real MVP authority preflight."""

import unittest
from unittest.mock import patch

from tools import check_real_mvp_authority as authority


class AuthorityDecisionTests(unittest.TestCase):
    def test_clean_snapshot_passes(self) -> None:
        self.assertEqual(authority.evaluate_authority(authority.AuthoritySnapshot()), ())

    def test_legacy_service_fails_closed(self) -> None:
        failures = authority.evaluate_authority(
            authority.AuthoritySnapshot(legacy_service_active=True)
        )
        self.assertTrue(any("legacy service active" in item for item in failures))

    def test_unexpected_and_duplicate_publishers_fail(self) -> None:
        failures = authority.evaluate_authority(
            authority.AuthoritySnapshot(
                topic_publishers={
                    "/scan_3d": ("rslidar_points_destination_0",),
                    "/cmd_vel_final": ("nav_a", "nav_b"),
                }
            )
        )
        self.assertTrue(any("unexpected publisher on /scan_3d" in item for item in failures))
        self.assertTrue(any("duplicate publishers on /cmd_vel_final" in item for item in failures))

    def test_uart_owner_and_relevant_process_fail(self) -> None:
        failures = authority.evaluate_authority(
            authority.AuthoritySnapshot(
                process_matches=("1234 /usr/bin/mavros_node",),
                device_owners={"/dev/ttyUSB0": ("5678 controller_serv",)},
            ),
            required_devices=("/dev/ttyUSB0",),
        )
        self.assertTrue(any("unexpected relevant process" in item for item in failures))
        self.assertTrue(any("device already owned" in item for item in failures))

    def test_tf_and_odometry_authority_are_covered(self) -> None:
        failures = authority.evaluate_authority(
            authority.AuthoritySnapshot(
                topic_publishers={
                    "/tf": ("robot_state_publisher",),
                    "/odometry/local": ("legacy_ekf",),
                }
            )
        )
        self.assertTrue(any("unexpected publisher on /tf" in item for item in failures))
        self.assertTrue(any("unexpected publisher on /odometry/local" in item for item in failures))


class RuntimeProbeTests(unittest.TestCase):
    def test_runtime_snapshot_snippet_compiles(self) -> None:
        compile(authority._RUNTIME_GRAPH_SNAPSHOT, "runtime_graph_snapshot", "exec")

    @patch("tools.check_real_mvp_authority._device_owners", return_value=())
    @patch("tools.check_real_mvp_authority._process_matches", return_value=())
    @patch("tools.check_real_mvp_authority._service_is_active", return_value=False)
    @patch("tools.check_real_mvp_authority._run")
    def test_collect_snapshot_allows_absent_topics_through_single_runtime_probe(
        self, run_mock, _service_mock, _process_mock, _devices_mock
    ) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            [str(authority.RUNTIME_EXEC)],
            0,
            '{"/cmd_vel_final": [], "/scan_3d": []}',
            "",
        )

        snapshot = authority.collect_snapshot(
            devices=(), topics=("/scan_3d", "/cmd_vel_final"), process_patterns=()
        )

        self.assertEqual(snapshot.topic_publishers["/scan_3d"], ())
        self.assertEqual(snapshot.topic_publishers["/cmd_vel_final"], ())
        self.assertEqual(authority.evaluate_authority(snapshot), ())
        command = run_mock.call_args.args[0]
        self.assertEqual(command[:4], (str(authority.RUNTIME_EXEC), "--", "bash", "-lc"))
        self.assertIn("python3 -c", command[4])
        self.assertNotIn("ros2 topic", command[4])
        self.assertIn("/scan_3d", command[4])
        self.assertIn("/cmd_vel_final", command[4])
        self.assertEqual(run_mock.call_count, 1)
        self.assertEqual(
            run_mock.call_args.kwargs["timeout_s"], authority.RUNTIME_GRAPH_TIMEOUT_S
        )

    @patch("tools.check_real_mvp_authority._run")
    def test_invalid_runtime_snapshot_fails_closed(self, run_mock) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            [str(authority.RUNTIME_EXEC)], 0, "not-json", ""
        )

        with self.assertRaises(authority.ProbeError):
            authority._runtime_topic_publishers(("/scan_3d",))

    @patch("tools.check_real_mvp_authority._run")
    def test_runtime_failure_fails_closed(self, run_mock) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            [str(authority.RUNTIME_EXEC)], 1, "", "runtime unavailable"
        )

        with self.assertRaises(authority.ProbeError):
            authority._runtime_topic_publishers(("/scan_3d",))

    @patch("tools.check_real_mvp_authority._run")
    def test_service_probe_is_read_only(self, run_mock) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            ["systemctl"], 3, "", ""
        )
        self.assertFalse(authority._service_is_active("salus-real-global-v2-wifi.service"))
        self.assertEqual(run_mock.call_args.args[0][:3], ("systemctl", "is-active", "--quiet"))


if __name__ == "__main__":
    unittest.main()
