from __future__ import annotations

import math

from elesim_pilot.remote_state import RemoteState
from elesim_protocol import SimMappingConfig


def test_measured_q_is_the_canonical_control_state() -> None:
    state = RemoteState(SimMappingConfig(), clock=lambda: 10.0)
    state.peer_connected(True)
    state.target_changed("robot-a")
    state.accept_telemetry(
        {"q": [-0.1, 0.2, 0.3, -0.4], "q_source": "measured", "torque_enabled": True}
    )

    snapshot = state.snapshot(tx_seq=7)
    assert snapshot.connected is True
    assert snapshot.tx_seq == 7
    assert snapshot.q is not None
    assert snapshot.q.linear_m == -0.1
    assert snapshot.u is not None
    assert snapshot.torque_enabled is True


def test_gripper_feedback_survives_operator_snapshot_and_target_switch():
    state = RemoteState(SimMappingConfig(), clock=lambda: 10.0)
    state.peer_connected(True)
    state.target_changed("robot-a")
    state.accept_telemetry({"claw_closed": True})
    assert state.snapshot(tx_seq=0).claw_closed is True
    state.accept_telemetry({"claw_closed": False})
    assert state.snapshot(tx_seq=0).claw_closed is False
    state.target_changed("sim-a")
    assert state.snapshot(tx_seq=0).claw_closed is None


def test_target_switch_clears_previous_robot_measurements() -> None:
    state = RemoteState(SimMappingConfig(), clock=lambda: 10.0)
    state.target_changed("robot-a")
    state.accept_telemetry({"q": [-0.1, 0.2, 0.3, -0.4]})
    state.target_changed("robot-b")

    snapshot = state.snapshot(tx_seq=0)
    assert snapshot.q is None
    assert snapshot.connected is False


def test_stale_telemetry_is_not_reported_as_connected() -> None:
    now = [10.0]
    state = RemoteState(SimMappingConfig(), stale_after_s=1.0, clock=lambda: now[0])
    state.peer_connected(True)
    state.target_changed("robot-a")
    state.accept_telemetry({"q": [0.0, 0.0, 0.0, 0.0]})
    now[0] = 11.1

    assert state.snapshot(tx_seq=0).connected is False


def test_snapshot_before_first_telemetry_uses_a_finite_unknown_age() -> None:
    state = RemoteState(SimMappingConfig(), clock=lambda: 10.0)

    snapshot = state.snapshot(tx_seq=0)

    assert snapshot.connected is False
    assert snapshot.rx_age_s == -1.0
    assert snapshot.host_state_age_s == -1.0
    assert math.isfinite(snapshot.rx_age_s)
    assert math.isfinite(snapshot.host_state_age_s)


def test_new_grant_same_endpoint_clears_only_obsolete_missing_lease_error():
    state = RemoteState(SimMappingConfig(), clock=lambda:10.)
    state.peer_connected(True)
    state.target_changed("robot-a")
    state.accept_telemetry({"q":[-.1,0,0,0],"q_source":"measured"})
    state.accept_error("motion command rejected: no_active_lease")
    state.accept_telemetry({"q":[-.1,0,0,0],"q_source":"measured"})
    assert not state.snapshot(tx_seq=0).reply_ok
    state.target_changed("robot-a")  # new lease, same Robot endpoint
    assert state.snapshot(tx_seq=0).reply_ok
    assert state.snapshot(tx_seq=0).reply_reason == ""
    state.accept_error("motor overcurrent")
    state.target_changed("robot-a")
    assert not state.snapshot(tx_seq=0).reply_ok
    assert state.snapshot(tx_seq=0).reply_reason == "motor overcurrent"
