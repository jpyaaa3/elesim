from types import SimpleNamespace

from elesim_protocol import CAPABILITY_ROBOT_STATE_MIRROR, DdsTransportError, PeerIdentity
from elesim_robot.main import publish_mirror_telemetry


def test_disappearing_viewer_cannot_abort_publication_or_mutate_motion_state():
    sent = []
    def discover(*, role, capability):
        assert (role, capability) == ("sim", CAPABILITY_ROBOT_STATE_MIRROR)
        return [SimpleNamespace(identity=PeerIdentity(name, "boot")) for name in ("lost-sim", "live-sim")]
    def send(kind, **kwargs):
        if kwargs["target_id"] == "lost-sim":
            raise DdsTransportError("viewer disappeared")
        sent.append((kind, kwargs))
    client = SimpleNamespace(node=SimpleNamespace(discover=discover), send=send)
    state = {"q": [-0.1, 0.2, 0.3, -0.3], "q_source": "measured", "torque_enabled": False}
    publish_mirror_telemetry(client, state)
    assert sent == [("telemetry", {"target_id": "live-sim", "payload": state})]
    assert state["torque_enabled"] is False
