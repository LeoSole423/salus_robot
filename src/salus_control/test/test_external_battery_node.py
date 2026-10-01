"""Exercise the real ROS controller adapter with an inert drive client."""
import json
import math
from types import SimpleNamespace

import pytest
import rclpy
from sensor_msgs.msg import BatteryState

import salus_control.controller_server_node as controller
from salus_control.rpy_esp32_comms.transport import CommsStats


class InertTransport:
    def start(self):
        pass

    def stop(self):
        pass

    def get_latest_telemetry(self):
        return None

    def get_latest_battery_telemetry(self):
        raise AssertionError('External source must never read ESP32 battery')

    def get_stats(self):
        return CommsStats()

    def get_command_state(self):
        return {}


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(controller, 'create_transport_backend', lambda **kw: InertTransport())
    monkeypatch.setattr(controller, 'resolve_serial_port',
                        lambda _: SimpleNamespace(port='/dev/inert', reason='test'))
    rclpy.init(args=['--ros-args', '-p', 'battery_source:=external'])
    instance = controller.ControllerServerNode()
    output = {}
    for attribute, key in [('_battery_state_pub', 'state'),
                           ('_battery_guard_pub', 'guard'), ('_telemetry_pub', 'json')]:
        setattr(instance, attribute,
                SimpleNamespace(publish=lambda msg, key=key: output.update({key: msg})))
    yield instance, output
    instance.destroy_node()
    rclpy.shutdown()


def sample(node):
    msg = BatteryState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.present, msg.voltage, msg.percentage = True, 49.8, 0.93
    msg.current, msg.capacity, msg.charge = -0.4, 50., 46.5
    return msg


def test_external_state_reaches_public_topics_and_cockpit(node):
    instance, output = node
    msg = sample(instance)
    instance._on_external_battery(msg)
    instance._telemetry_tick()
    assert output['state'].percentage == pytest.approx(.93)
    assert output['state'].current == pytest.approx(-.4)
    assert output['state'].header.stamp == msg.header.stamp
    assert output['guard'].fresh
    assert output['guard'].operator_soc_pct == pytest.approx(93)
    payload = json.loads(output['json'].data)['battery']
    assert payload['percentage'] == pytest.approx(.93)
    assert payload['soc_model'] == 'bms_capacity_ratio'


def test_stale_data_is_unknown_and_never_falls_back(node):
    instance, output = node
    instance._on_external_battery(sample(instance))
    instance._external_battery = controller.external_battery_sample(
        instance._external_battery.message, controller.time.monotonic() - 10)
    instance._telemetry_tick()
    assert not output['state'].present
    assert math.isnan(output['state'].percentage)
    assert not output['guard'].fresh
    assert output['guard'].state == 'STALE'
    assert json.loads(output['json'].data)['battery']['percentage'] is None


def test_startup_without_bms_is_unavailable(node):
    instance, output = node
    instance._telemetry_tick()
    assert not output['state'].present
    assert math.isnan(output['state'].voltage)
    assert not output['guard'].ready
    assert output['guard'].state == 'UNAVAILABLE'


def test_duplicate_stamp_does_not_refresh_sample(node):
    instance, _ = node
    msg = sample(instance)
    instance._on_external_battery(msg)
    first = instance._external_battery
    instance._on_external_battery(msg)
    assert instance._external_battery is first


def test_invalid_sample_does_not_clear_home_latch(node):
    instance, output = node
    instance._battery_estimator.update(46., sample_time_s=0, traction_active=False)
    instance._battery_estimator.update(46., sample_time_s=30, traction_active=False)
    msg = sample(instance)
    msg.percentage = math.nan
    instance._on_external_battery(msg)
    instance._telemetry_tick()
    assert output['guard'].return_home_recommended
    assert not output['guard'].ready
