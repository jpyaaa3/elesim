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


def test_no_discovered_robot_lost_peer_and_wrong_role_leave_pose_unchanged():
    for blocked in ("disabled", "lost", "wrong-role"):
        endpoint, state, peer, client, message = setup_mirror()
        before = state.estimate_q()
        if blocked == "disabled":
            endpoint.mirror_robot_id = ""
            client.node.discover = lambda **_: []
        elif blocked == "lost":
            client.node.resolve = lambda _: None
        elif blocked == "wrong-role":
            peer.descriptor = EndpointDescriptor("robot-a", "pilot")
        endpoint.handle_envelope(client, message)
        assert state.estimate_q() == before


def test_robot_mirror_takes_display_ownership_even_with_a_previous_sim_lease():
    endpoint, state, peer, client, message = setup_mirror()
    endpoint.grant_lease("pilot-a", "lease-a")
    endpoint.handle_envelope(client, message)
    assert state.estimate_q().roll_rad == 0.2
    assert endpoint._apply_motion(Envelope(
        "motion_command", "pilot-a", "sim-a", lease_id="lease-a", payload={"command": "target", "q": [0] * 4},
    )) == (False, "robot_present_sim_is_presentation_only")


def test_discovered_robot_blocks_sim_motion_before_first_telemetry_and_after_loss():
    endpoint, state, robot, client, telemetry = setup_mirror()
    endpoint.mirror_robot_id = ""
    endpoint.grant_lease("pilot-a", "lease-a")
    client.node.discover = lambda **kwargs: [robot]
    replies = []
    client.send = lambda *args, **kwargs: replies.append(kwargs["payload"])
    motion = Envelope("motion_command", "pilot-a", "sim-a", lease_id="lease-a",
        payload={"command": "target", "q": [-.1, .3, .1, -.1]}, seq=1)
    before = state.estimate_q()
    endpoint.handle_envelope(client, motion)
    assert replies[-1]["reason"] == "robot_present_sim_is_presentation_only"
    client.node.discover = lambda **kwargs: []
    client.node.resolve = lambda identity: None
    endpoint.handle_envelope(client, replace(motion, seq=2))
    assert replies[-1]["ok"] is False
    assert state.estimate_q() == before


def test_selected_robot_route_rejects_wrong_pilot_boot_and_reordered_updates():
    endpoint, state, robot, client, telemetry = setup_mirror()
    endpoint = SimEndpoint(endpoint_id='sim-a', state=state, streams={})
    client.node.discover = lambda **kwargs: [robot]
    endpoint._select_discovered_robot(client)
    pilot = SimpleNamespace(descriptor=EndpointDescriptor("pilot-a", "pilot"),
                            identity=PeerIdentity("pilot-a", "pilot-boot"))
    other = SimpleNamespace(descriptor=EndpointDescriptor("robot-b", "robot"),
                            identity=PeerIdentity("robot-b", "boot-b"))
    directory = {"pilot-a": pilot, "robot-a": robot, "robot-b": other}
    client.node.resolve = directory.get
    route = Envelope("presentation_target", "pilot-a", "sim-a", seq=2, source_boot_id="pilot-boot",
                     payload={"robot_id": "robot-b", "robot_boot_id": "boot-b"})
    endpoint.handle_envelope(client, replace(route, source_boot_id="old-pilot"))
    assert endpoint.mirror_robot_id == "robot-a"
    endpoint.handle_envelope(client, route)
    assert endpoint.mirror_robot_id == "robot-b"
    endpoint.handle_envelope(client, replace(route, seq=1, payload={"robot_id": "robot-a", "robot_boot_id": "boot-a"}))
    endpoint.handle_envelope(client, telemetry)
    assert endpoint.mirror_robot_id == "robot-b"
    assert state.estimate_q().roll_rad != 0.2
    endpoint.handle_envelope(client, replace(telemetry, source_id="robot-b", source_boot_id="boot-b"))
    assert state.estimate_q().roll_rad == 0.2


def test_explicit_mirror_robot_pin_cannot_be_replaced_by_pilot_selection():
    endpoint, state, robot, client, telemetry = setup_mirror()
    pilot = SimpleNamespace(descriptor=EndpointDescriptor('pilot-a', 'pilot'),
                            identity=PeerIdentity('pilot-a', 'pilot-boot'))
    other = SimpleNamespace(descriptor=EndpointDescriptor('robot-b', 'robot'),
                            identity=PeerIdentity('robot-b', 'boot-b'))
    client.node.resolve = {'pilot-a': pilot, 'robot-a': robot, 'robot-b': other}.get
    route = Envelope('presentation_target', 'pilot-a', 'sim-a', seq=2, source_boot_id='pilot-boot',
        payload={'robot_id': 'robot-b', 'robot_boot_id': 'boot-b'})
    endpoint.handle_envelope(client, route)
    assert endpoint.mirror_robot_id == 'robot-a'
    endpoint.handle_envelope(client, replace(telemetry, source_id='robot-b', source_boot_id='boot-b'))
    assert state.estimate_q().roll_rad != .2
    endpoint.handle_envelope(client, telemetry)
    assert state.estimate_q().roll_rad == .2
    robot.identity = PeerIdentity('robot-a', 'boot-new')
    endpoint.handle_envelope(client, replace(route, seq=3,
        payload={'robot_id': 'robot-a', 'robot_boot_id': 'boot-new'}))
    assert endpoint._mirror_expected_boot == 'boot-new'
    endpoint.handle_envelope(client, replace(telemetry, seq=1, source_boot_id='boot-new',
        payload={**telemetry.payload, 'q': [-.1, .4, .3, -.3]}))
    assert state.estimate_q().roll_rad == .4


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
