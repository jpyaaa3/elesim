from types import SimpleNamespace

from elesim_protocol import (
    CAPABILITY_ROBOT_STATE_MIRROR,
    DdsTransportError,
    PeerAmbiguityError,
    PeerIdentity,
)
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


def test_ambiguous_sim_identity_skips_mirror_without_stopping_robot(capsys):
    peer = SimpleNamespace(identity=PeerIdentity("sim-default", "boot-a"))

    def discover(*, role, capability):
        assert (role, capability) == ("sim", CAPABILITY_ROBOT_STATE_MIRROR)
        return [peer]

    def send(_kind, **_kwargs):
        raise PeerAmbiguityError(
            "sim-default",
            (PeerIdentity("sim-default", "boot-a"), PeerIdentity("sim-default", "boot-b")),
        )

    client = SimpleNamespace(node=SimpleNamespace(discover=discover), send=send)
    publish_mirror_telemetry(client, {"q_source": "measured"})

    assert "multiple active boot IDs" in capsys.readouterr().out
