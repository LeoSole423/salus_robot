"""Isolated ROS runtime proof for the physical Pixhawk RTCM delivery profile."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from mavros_msgs.msg import GPSRAW, RTCM
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from salus_interfaces.msg import GnssRtkStatus, RtcmFrame

from salus_hardware.rtk_domain import crc24q


SOURCE_STATUS_TOPIC = "/salus/hardware/gnss_primary/rtk_source_status"
RTCM_INPUT_TOPIC = "/salus/hardware/rtcm/corrections"
GPSRAW_TOPIC = "/mavros_node/mavros_node/gps1/raw"
STATUS_TOPIC = "/salus/hardware/gnss_primary/rtk_status"
MAVROS_RTCM_TOPIC = "/mavros_node/mavros_node/send_rtcm"
ADAPTER_NODE = "pixhawk_rtk_adapter"


class DeliveryRuntimeHarness(Node):
    """Publish synthetic inputs and capture only the adapter's outputs."""

    def __init__(self) -> None:
        super().__init__("pixhawk_rtk_delivery_real_runtime_probe")
        self.deliveries: list[RTCM] = []
        self.statuses: list[GnssRtkStatus] = []
        self.rtcm_publisher = self.create_publisher(RtcmFrame, RTCM_INPUT_TOPIC, 10)
        self.gpsraw_publisher = self.create_publisher(GPSRAW, GPSRAW_TOPIC, 10)
        self.source_status_publisher = self.create_publisher(
            GnssRtkStatus, SOURCE_STATUS_TOPIC, 10
        )
        self.create_subscription(RTCM, MAVROS_RTCM_TOPIC, self.deliveries.append, 10)
        self.create_subscription(GnssRtkStatus, STATUS_TOPIC, self.statuses.append, 10)

    def wait_for(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()


def _rtcm_frame(sequence: int, payload: bytes = b"\x3e\x00") -> RtcmFrame:
    prefix = bytes(
        (0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF)
    ) + payload
    message = RtcmFrame()
    message.source_id = "synthetic_base"
    message.sequence = sequence
    message.data = list(prefix + crc24q(prefix).to_bytes(3, "big"))
    return message


def _invalid_frame(sequence: int) -> RtcmFrame:
    message = _rtcm_frame(sequence)
    message.data[-1] ^= 1
    return message


def _oversized_frame(sequence: int) -> RtcmFrame:
    return _rtcm_frame(sequence, bytes(715))


def _start_launch(log_path: Path) -> subprocess.Popen:
    with log_path.open("w", encoding="utf-8") as log_file:
        return subprocess.Popen(
            [
                "ros2",
                "launch",
                "salus_hardware",
                "pixhawk_rtk_delivery_real.launch.py",
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover - defensive cleanup
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10)


def _node_names(harness: DeliveryRuntimeHarness) -> set[str]:
    return {name for name, _namespace in harness.get_node_names_and_namespaces()}


def _endpoint_names(infos) -> set[str]:
    return {info.node_name for info in infos}


def test_physical_delivery_profile_isolated_and_fail_closed(tmp_path) -> None:
    rclpy.init()
    harness = DeliveryRuntimeHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "pixhawk_rtk_delivery_real.log"
    forbidden_nodes = {
        "mavros_node",
        "ntrip_rtcm_source",
        "rslidar_sdk",
        "pixhawk_sensor_adapter",
        "robot_state_publisher",
        "ekf_filter_node_map",
        "ekf_filter_node_odom",
        "controller_server",
        "nav2_controller",
        "cockpit",
    }
    try:
        process = _start_launch(log_path)
        assert harness.wait_for(
            lambda: ADAPTER_NODE in _node_names(harness), timeout_s=20.0
        ), f"adapter never joined the graph; log:\n{log_path.read_text()[-3000:]}"
        assert process.poll() is None, log_path.read_text()

        node_names = [
            name for name, _namespace in harness.get_node_names_and_namespaces()
        ]
        assert node_names.count(ADAPTER_NODE) == 1
        assert not set(node_names).intersection(forbidden_nodes)

        endpoints_ready = harness.wait_for(
            lambda: (
                _endpoint_names(
                    harness.get_subscriptions_info_by_topic(SOURCE_STATUS_TOPIC)
                )
                == {ADAPTER_NODE}
                and _endpoint_names(
                    harness.get_subscriptions_info_by_topic(RTCM_INPUT_TOPIC)
                )
                == {ADAPTER_NODE}
                and _endpoint_names(
                    harness.get_subscriptions_info_by_topic(GPSRAW_TOPIC)
                )
                == {ADAPTER_NODE}
                and _endpoint_names(
                    harness.get_publishers_info_by_topic(STATUS_TOPIC)
                )
                == {ADAPTER_NODE}
                and _endpoint_names(
                    harness.get_publishers_info_by_topic(MAVROS_RTCM_TOPIC)
                )
                == {ADAPTER_NODE}
            ),
            timeout_s=20.0,
        )
        assert endpoints_ready, (
            "physical endpoints were not advertised by the single adapter; "
            f"log:\n{log_path.read_text()[-3000:]}"
        )

        publisher_types = {
            info.topic_type
            for info in harness.get_publishers_info_by_topic(MAVROS_RTCM_TOPIC)
        }
        assert publisher_types == {"mavros_msgs/msg/RTCM"}

        harness.rtcm_publisher.publish(_rtcm_frame(1))
        assert harness.wait_for(lambda: len(harness.deliveries) == 1, timeout_s=5.0)
        assert bytes(harness.deliveries[0].data) == bytes(_rtcm_frame(1).data)

        harness.rtcm_publisher.publish(_rtcm_frame(1))
        harness.rtcm_publisher.publish(_invalid_frame(2))
        harness.rtcm_publisher.publish(_oversized_frame(3))
        assert not harness.wait_for(lambda: len(harness.deliveries) > 1, timeout_s=1.5)
        assert len(harness.deliveries) == 1

        gps = GPSRAW()
        gps.fix_type = GPSRAW.GPS_FIX_TYPE_RTK_FIXED
        gps.satellites_visible = 24
        harness.gpsraw_publisher.publish(gps)
        assert harness.wait_for(
            lambda: any(
                status.fix_quality == GnssRtkStatus.RTK_FIXED
                and status.receiver_fix_type == 6
                for status in harness.statuses
            ),
            timeout_s=5.0,
        )

        source = GnssRtkStatus()
        source.source_id = "synthetic_base"
        source.acquisition_state = GnssRtkStatus.ACQUISITION_RECEIVING
        source.corrections_fresh = True
        source.correction_age_s = 0.0
        source.received_count = 1
        unknown_gps = GPSRAW()
        unknown_gps.fix_type = 9
        harness.source_status_publisher.publish(source)
        harness.gpsraw_publisher.publish(unknown_gps)
        assert harness.wait_for(
            lambda: any(
                status.fix_quality == GnssRtkStatus.UNKNOWN
                and status.corrections_fresh
                for status in harness.statuses
            ),
            timeout_s=5.0,
        )

        status_count = len(harness.statuses)
        stale_deadline = time.monotonic() + 5.5
        assert harness.wait_for(
            lambda: (
                time.monotonic() >= stale_deadline
                and len(harness.statuses) > status_count
                and harness.statuses[-1].fix_quality == GnssRtkStatus.UNKNOWN
                and harness.statuses[-1].receiver_fix_type == -1
            ),
            timeout_s=7.0,
        )
    finally:
        if process is not None:
            _stop(process)
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        spinner.join(timeout=5.0)
        rclpy.shutdown()
