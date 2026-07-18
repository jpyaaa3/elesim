from __future__ import annotations

from apps.server.main import RouterCore
from engine.core.protocol import EndpointDescriptor, make_envelope


def _register(core: RouterCore, identity: bytes, endpoint_id: str, role: str, seq: int = 1) -> None:
    descriptor = EndpointDescriptor(endpoint_id, role)
    core.handle(
        identity,
        make_envelope("register", endpoint_id, payload={"endpoint": descriptor.to_dict()}, seq=seq),
        now=1.0,
    )


def test_pick_command_sources_are_metadata_under_controller_lease() -> None:
    core = RouterCore()
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    selected = core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "robot-a"}, seq=2),
        now=1.1,
    )[-1].envelope
    lease_id = str(selected.payload["lease_id"])
    for seq, source in enumerate(("lji", "lji_step", "servo", "ik", "experiment"), start=3):
        routed = core.handle(
            b"controller",
            make_envelope(
                "command",
                "controller-a",
                target_id="robot-a",
                payload={"command": "target", "source": source},
                seq=seq,
                lease_id=lease_id,
            ),
            now=1.1 + seq * 0.01,
        )
        assert routed[0].identity == b"robot"
        assert routed[0].envelope.payload["source"] == source

