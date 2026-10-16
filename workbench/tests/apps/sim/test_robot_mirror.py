from dataclasses import replace
from types import SimpleNamespace

import pytest

from elesim_protocol import EndpointDescriptor, Envelope, PeerIdentity, SimMappingConfig
from elesim_sim.control_state import SimulationStateSource
from elesim_sim.endpoint import SimEndpoint


def setup_mirror():
    state = SimulationStateSource(SimMappingConfig())
    endpoint = SimEndpoint(endpoint_id="sim-a", state=state, streams={}, mirror_robot_id="robot-a")
    peer = SimpleNamespace(descriptor=EndpointDescriptor("robot-a", "robot"),
                           identity=PeerIdentity("robot-a", "boot-a"))
    client = SimpleNamespace(node=SimpleNamespace(resolve=lambda _id: peer))
    message = Envelope("telemetry", "robot-a", "sim-a", seq=10, source_boot_id="boot-a",
                       payload={"q": [-0.1, 0.2, 0.3, -0.3], "q_source": "measured"})
    return endpoint, state, peer, client, message


def test_measured_pose_needs_no_motion_lease_and_never_sends_commands():
    endpoint, state, _peer, client, message = setup_mirror()
    endpoint.handle_envelope(client, message)
    assert state.estimate_q().roll_rad == 0.2
    assert not endpoint.active_lease
    # Client intentionally has no send method: this display path cannot actuate.
    assert state.go2_vel() == (0.0, 0.0, 0.0)
    assert state.robot_mirror_active
    state.apply_target({"command": "target", "q": [-0.1, 0, 0, 0]})
    assert not state.robot_mirror_active


def test_mirror_uses_measured_gripper_instead_of_default_open_state():
    endpoint, state, _peer, client, message = setup_mirror()
    endpoint.handle_envelope(client, replace(message, payload={**message.payload, "claw_closed": True}))
    assert state.claw_closed() is True
    endpoint.handle_envelope(client, replace(message, seq=11, payload={**message.payload, "claw_closed": False}))
    assert state.claw_closed() is False


@pytest.mark.parametrize("changes", [
    {"source_id": "other-robot"}, {"source_boot_id": "old-boot"},
    {"source_boot_id": ""}, {"target_id": "other-sim"},
    {"payload": {"q": [-0.1, 0.4, 0.3, -0.3]}},
    {"payload": {"q_source": "estimated", "q": [-0.1, 0.4, 0.3, -0.3]}},
    {"payload": {"q_source": "measured", "q": [-0.1, 0.4]}},
    {"payload": {"q_source": "measured", "q": [-0.1, True, 0.3, -0.3]}},
])
def test_untrusted_or_unmeasured_state_does_not_change_pose(changes):
    endpoint, state, _peer, client, message = setup_mirror()
    before = state.estimate_q()
    endpoint.handle_envelope(client, replace(message, **changes))
    assert state.estimate_q() == before


def test_sequence_replay_old_boot_and_new_robot_boot():
    endpoint, state, peer, client, message = setup_mirror()
    endpoint.handle_envelope(client, message)
    changed = {"q_source": "measured", "q": [-0.1, 0.5, 0.3, -0.3]}
    endpoint.handle_envelope(client, replace(message, seq=9, payload=changed))
    assert state.estimate_q().roll_rad == 0.2
    peer.identity = PeerIdentity("robot-a", "boot-b")
    endpoint.handle_envelope(client, replace(message, seq=20, payload=changed))
    assert state.estimate_q().roll_rad == 0.2
    endpoint.handle_envelope(client, replace(message, source_boot_id="boot-b", seq=1, payload=changed))
    assert state.estimate_q().roll_rad == 0.5


def test_disabled_mirror_lost_peer_wrong_role_and_sim_control_take_precedence():
    for blocked in ("disabled", "lost", "wrong-role", "lease"):
        endpoint, state, peer, client, message = setup_mirror()
        before = state.estimate_q()
        if blocked == "disabled":
            endpoint.mirror_robot_id = ""
        elif blocked == "lost":
            client.node.resolve = lambda _: None
        elif blocked == "wrong-role":
            peer.descriptor = EndpointDescriptor("robot-a", "pilot")
        else:
            endpoint.grant_lease("pilot-a", "lease-a")
        endpoint.handle_envelope(client, message)
        assert state.estimate_q() == before


def test_latest_snapshot_clears_missing_go2_data_and_rejects_partial_invalid_updates():
    endpoint, state, _peer, client, message = setup_mirror()
    state.update_go2_mirror(base_pos=(1, 2, 3), leg_q=(0,) * 12)
    endpoint.handle_envelope(client, message)
    assert state.go2_base_pos() is None
    assert state.go2_leg_q() is None
    before = state.estimate_q()
    endpoint.handle_envelope(client, replace(message, seq=11, payload={
        "q_source": "measured", "q": [-0.1, 0.5, 0.3, -0.3], "go2_leg_q": [0],
    }))
    assert state.estimate_q() == before


def test_available_go2_pose_is_displayed_without_forwarding_velocity_or_torque():
    endpoint, state, _peer, client, message = setup_mirror()
    endpoint.handle_envelope(client, replace(message, payload={
        **message.payload, "torque_enabled": True,
        "go2": {"position": [1, 2, 0.3], "rpy": [0, 0, 0.4]},
        "go2_velocity": [1, 1, 1],
    }))
    assert state.go2_base_pos() == (1.0, 2.0, 0.3)
    assert state.go2_base_rpy() == (0.0, 0.0, 0.4)
    assert state.go2_vel() == (0.0, 0.0, 0.0)
