from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from elesim_robot.runtime import RobotRuntime
from elesim_protocol import EndpointDescriptor, SimMappingConfig, make_envelope


def _runtime(*, hardware=None, go2=None) -> RobotRuntime:
    runtime = RobotRuntime(mapping=SimMappingConfig(), hardware_config=SimpleNamespace(), go2_bridge=go2)
    runtime.hw = hardware
    runtime.grant_lease("controller-a", "lease-a")
    return runtime


def _command(seq: int, payload: dict):
    return make_envelope(
        "motion_command",
        "controller-a",
        target_id="robot-a",
        payload=payload,
        seq=seq,
        lease_id="lease-a",
    )


def test_go2_velocity_is_executed_by_robot_agent() -> None:
    go2 = MagicMock()
    runtime = _runtime(go2=go2)
    go2.reset_mock()
    ok, reason = runtime.apply(_command(1, {"command": "go2_velocity", "vx": 0.2, "vy": 0.1, "wz": -0.3}))
    assert ok and reason == "go2_velocity"
    go2.set_velocity.assert_called_once_with(0.2, 0.1, -0.3)


def test_go2_target_metadata_is_executed_by_robot_agent() -> None:
    go2 = MagicMock()
    runtime = _runtime(go2=go2)
    go2.reset_mock()
    ok, reason = runtime.apply(
        _command(
            1,
            {
                "command": "target",
                "go2_sport_pose": "stand_up",
                "go2_obstacles_avoid_enable": True,
            },
        )
    )
    assert ok and reason == "target_meta"
    go2.call_sport_pose.assert_called_once_with("stand_up")
    go2.set_obstacles_avoid.assert_called_once_with(True)


def test_target_is_converted_and_sent_to_hardware() -> None:
    hardware = MagicMock()
    runtime = _runtime(hardware=hardware)
    ok, reason = runtime.apply(
        _command(
            1,
            {
                "command": "target",
                "q": [-0.1, 0.1, 0.2, -0.2],
            },
        )
    )
    assert ok and reason == "target"
    hardware.command_4dof_deg.assert_called_once()


def test_stale_sequence_and_wrong_lease_are_rejected() -> None:
    runtime = _runtime()
    assert runtime.apply(_command(2, {"command": "target"}))[0]
    assert runtime.apply(_command(2, {"command": "target"})) == (False, "stale_sequence")
    wrong = make_envelope(
        "motion_command", "controller-a", target_id="robot-a", payload={"command": "target"}, seq=3, lease_id="wrong"
    )
    assert runtime.apply(wrong) == (False, "lease_mismatch")


def test_lease_revoke_stops_local_motion() -> None:
    go2 = MagicMock()
    runtime = _runtime(go2=go2)
    runtime.revoke_lease()
    go2.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert runtime.active_lease == ""


def test_estop_bypasses_lease_and_disables_torque() -> None:
    hardware = MagicMock()
    runtime = _runtime(hardware=hardware)
    runtime.torque_enabled = True
    estop = make_envelope(
        "motion_command",
        "controller-other",
        target_id="robot-a",
        payload={"command": "estop"},
        seq=1,
    )
    assert runtime.apply(estop) == (True, "estop")
    hardware.torque_off_all.assert_called_once()
    assert runtime.torque_enabled is False


def test_robot_agent_descriptor_has_no_simulation_capability() -> None:
    descriptor = EndpointDescriptor("robot-a", "robot", ("arm", "go2", "rgb", "depth"))
    assert "rendered_view" not in descriptor.capabilities
