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


class ProbeParsingTests(unittest.TestCase):
    def test_topic_info_parser_handles_unknown_and_multiple_publishers(self) -> None:
        self.assertEqual(authority.parse_topic_publishers("Unknown topic '/scan_3d'"), ())
        output = """Type: sensor_msgs/msg/PointCloud2

Publisher count: 2

Node name: first_owner
Node namespace: /
Endpoint type: PUBLISHER

Node name: second_owner
Node namespace: /
Endpoint type: PUBLISHER

Subscription count: 0
"""
        self.assertEqual(
            authority.parse_topic_publishers(output),
            ("first_owner", "second_owner"),
        )

    @patch("tools.check_real_mvp_authority._device_owners", return_value=())
    @patch("tools.check_real_mvp_authority._process_matches", return_value=())
    @patch("tools.check_real_mvp_authority._service_is_active", return_value=False)
    @patch("tools.check_real_mvp_authority._run")
    def test_collect_snapshot_allows_absent_topic_through_full_probe(
        self, run_mock, _service_mock, _process_mock, _devices_mock
    ) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            [str(authority.RUNTIME_EXEC)], 1, "", "Unknown topic '/scan_3d'"
        )

        snapshot = authority.collect_snapshot(
            devices=(), topics=("/scan_3d",), process_patterns=()
        )

        self.assertEqual(snapshot.topic_publishers["/scan_3d"], ())
        self.assertEqual(authority.evaluate_authority(snapshot), ())
        command = run_mock.call_args.args[0]
        self.assertEqual(command[:4], (str(authority.RUNTIME_EXEC), "--", "bash", "-lc"))
        self.assertIn("ros2 topic info", command[4])

    @patch("tools.check_real_mvp_authority._run")
    def test_service_probe_is_read_only(self, run_mock) -> None:
        run_mock.return_value = authority.subprocess.CompletedProcess(
            ["systemctl"], 3, "", ""
        )
        self.assertFalse(authority._service_is_active("salus-real-global-v2-wifi.service"))
        self.assertEqual(run_mock.call_args.args[0][:3], ("systemctl", "is-active", "--quiet"))


if __name__ == "__main__":
    unittest.main()
