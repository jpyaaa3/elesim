from __future__ import annotations

from elesim_router.main import RouterCore
from elesim_protocol import CAPABILITY_MOTION_ARM, EndpointDescriptor, make_envelope


def _register(core: RouterCore, identity: bytes, endpoint_id: str, role: str, seq: int = 1) -> None:
    descriptor = EndpointDescriptor(endpoint_id, role, capabilities=(CAPABILITY_MOTION_ARM,))
    routed = core.handle(
        identity,
        make_envelope(
            "register",
            endpoint_id,
            payload={"endpoint": descriptor.to_dict()},
            seq=seq,
        ),
        now=1.0,
    )
    assert routed[0].envelope.message_type == "registered"


def test_robot_and_sim_register_and_are_listed() -> None:
    core = RouterCore()
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    _register(core, b"sim", "sim-a", "simulator")
    reply = core.handle(
        b"controller",
        make_envelope("discover", "controller-a", seq=2),
        now=1.1,
    )[0].envelope
    ids = {entry["endpoint_id"] for entry in reply.payload["endpoints"]}
    assert ids == {"robot-a", "sim-a"}


def test_active_lease_routes_only_matching_target() -> None:
    core = RouterCore()
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    selected = core.handle(
        b"controller",
        make_envelope(
            "select_target",
            "controller-a",
            payload={"target_id": "robot-a"},
            seq=2,
        ),
        now=1.1,
    )[-1].envelope
    lease_id = selected.payload["lease_id"]
    routed = core.handle(
        b"controller",
        make_envelope(
            "motion_command",
            "controller-a",
            target_id="robot-a",
            payload={"command": "target"},
            seq=3,
            lease_id=lease_id,
        ),
        now=1.2,
    )
    assert routed[0].identity == b"robot"
    rejected = core.handle(
        b"controller",
        make_envelope(
            "motion_command",
            "controller-a",
            target_id="robot-a",
            payload={"command": "target"},
            seq=4,
            lease_id="wrong",
        ),
        now=1.3,
    )
    assert rejected[0].envelope.message_type == "error"


def test_switch_revokes_old_target_before_new_lease() -> None:
    core = RouterCore()
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    _register(core, b"sim", "sim-a", "simulator")
    core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "robot-a"}, seq=2),
        now=1.1,
    )
    switched = core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "sim-a"}, seq=3),
        now=1.2,
    )
    assert switched[0].identity == b"robot"
    assert switched[0].envelope.message_type == "lease_revoked"
    assert switched[-1].envelope.payload["target_id"] == "sim-a"


def test_heartbeat_expiry_notifies_controller() -> None:
    core = RouterCore(heartbeat_timeout_s=1.0)
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "robot-a"}, seq=2),
        now=1.1,
    )
    core.handle(b"controller", make_envelope("heartbeat", "controller-a", seq=3), now=2.0)
    expired = core.expire(now=2.2)
    assert expired[0].identity == b"controller"
    assert expired[0].envelope.message_type == "target_lost"


def test_estop_routes_without_an_active_lease() -> None:
    core = RouterCore()
    _register(core, b"controller", "controller-a", "controller")
    _register(core, b"robot", "robot-a", "robot")
    routed = core.handle(
        b"controller",
        make_envelope(
            "motion_command",
            "controller-a",
            target_id="robot-a",
            payload={"command": "estop"},
            seq=2,
        ),
        now=1.1,
    )
    assert routed[0].identity == b"robot"
    assert routed[0].envelope.payload["command"] == "estop"
