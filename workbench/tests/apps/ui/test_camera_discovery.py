"""A Robot-free SROS2 graph must not create unauthorized camera participants."""
from types import SimpleNamespace
import threading

from elesim_ui.peer_runtime import UiPeerHub
from elesim_ui.zed_preview import ZedCameraPreview


def test_camera_waits_for_robot_discovery_before_creating_dds_subscriber():
    available = threading.Event()
    checked = threading.Event()
    subscribed = threading.Event()
    closed = threading.Event()
    calls = []

    def source_available():
        checked.set()
        return available.is_set()

    def factory(*_args, **_kwargs):
        calls.append("created")
        subscribed.set()
        def receive(**_kwargs):
            closed.wait(.02)
            return None
        return SimpleNamespace(connect=lambda: None,
                               recv_latest=receive,
                               close=closed.set)

    preview = ZedCameraPreview("/test/robot/rgbd/frame", endpoint_id="ui",
        settings=SimpleNamespace(), subscriber_factory=factory,
        source_available=source_available)
    preview.start()
    try:
        assert checked.wait(1)
        assert not subscribed.is_set()
        assert preview.status == "WAIT"
        assert preview.last_error == ""
        available.set()
        assert subscribed.wait(1)
        assert calls == ["created"]
    finally:
        preview.close()
    assert closed.is_set()


def test_robot_discovery_does_not_depend_on_pilot_or_control_target():
    roles = []
    def discover(*, role):
        roles.append(role)
        return [SimpleNamespace()] if role == "robot" else []
    hub = UiPeerHub(endpoint_id="ui", client=SimpleNamespace(
        node=SimpleNamespace(discover=discover)), autostart=False)
    assert hub.has_role("robot")
    assert not hub.has_role("sim")
    assert roles == ["robot", "sim"]
