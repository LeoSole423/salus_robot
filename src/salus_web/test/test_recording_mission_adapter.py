from types import SimpleNamespace
from unittest.mock import Mock

from salus_interfaces.srv import GetRouteMissionState
from salus_web.ros_gateway import CockpitRosGateway


def test_timeout_of_one_authority_keeps_known_paused_route_recording(monkeypatch):
    response = GetRouteMissionState.Response()
    response.ok = True
    response.active = False
    response.paused = True
    ready = Mock()
    ready.done.return_value = True
    ready.result.return_value = response
    stalled = Mock()
    stalled.done.return_value = False
    client = Mock()
    client.service_is_ready.return_value = True
    subject = SimpleNamespace(_recording_futures=[ready, stalled],
        _recording_last_poll=0, _recordings=Mock(),
        _service_clients={"get_route_state": client, "get_patrol_state": client})
    monkeypatch.setattr('salus_web.ros_gateway.time.monotonic', lambda: 10)
    CockpitRosGateway._poll_recording_mission(subject)
    stalled.cancel.assert_called_once()
    subject._recordings.observe.assert_called_once_with(True)


def test_unavailable_authorities_never_invent_idle_state():
    client = Mock()
    client.service_is_ready.return_value = False
    subject = SimpleNamespace(_recording_futures=None, _recordings=Mock(),
        _service_clients={"get_route_state": client, "get_patrol_state": client})
    CockpitRosGateway._poll_recording_mission(subject)
    subject._recordings.observe.assert_called_once_with(None)
