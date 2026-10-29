from __future__ import annotations

import pytest

from elesim_pilot.pick import ControlClient, ControlService, PanelState
from elesim_protocol import SimMappingConfig, SimQ


class Sender:
    def __init__(self) -> None:
        self.messages: list[tuple[dict[str, object], bool]] = []

    def __call__(self, message: dict[str, object], *, force: bool = False) -> None:
        self.messages.append((message, force))


def test_client_is_disconnected_until_target_telemetry_arrives() -> None:
    client = ControlClient(cfg=SimMappingConfig())
    assert client.get_state().connected is False

    client.peer_connected(True)
    client.target_changed("robot-a")
    client.accept_telemetry({"q": [0.0, 0.0, 0.0, 0.0]})
    assert client.get_state().connected is True


def test_partial_display_update_sends_one_full_canonical_q() -> None:
    mapping = SimMappingConfig()
    sender = Sender()
    client = ControlClient(cfg=mapping)
    client.attach_sender(sender)
    service = ControlService(
        PanelState(),
        client=client,
        mapping_cfg=mapping,
    )

    service.apply_partial_control_u({"roll": 200.0, "s1": 30.0}, source="slider")

    message, force = sender.messages[-1]
    assert message["t"] == "target"
    assert message["source"] == "slider"
    assert isinstance(message["q"], list)
    assert len(message["q"]) == 4
    assert "u" not in message
    assert force is False


def test_target_commands_are_never_dropped_inside_workflow_client() -> None:
    sender = Sender()
    client = ControlClient(cfg=SimMappingConfig())
    client.attach_sender(sender)

    client.send_target_values(linear_m=-0.1, roll_rad=0.0, theta1_rad=0.1, theta2_rad=-0.1)
    client.send_target_values(linear_m=-0.2, roll_rad=0.1, theta1_rad=0.2, theta2_rad=-0.2)

    assert len(sender.messages) == 2
    assert sender.messages[-1][0]["q"] == [-0.2, 0.1, 0.2, -0.2]


def test_final_mock_hug_target_carries_stale_plan_fence() -> None:
    sender = Sender()
    client = ControlClient(cfg=SimMappingConfig())
    client.attach_sender(sender)
    solution = type(
        "Solution",
        (),
        {
            "solution_id": "hug-1",
            "object_revision": 2,
            "object_sha256": "a" * 64,
            "final_q": (-0.1, 0.0, 0.2, 0.2),
        },
    )()
    client.send_mock_hug_target(
        q=SimQ(*solution.final_q),
        solution=solution,
        execution_context=("sim-a", "boot-a", "lease-a"),
    )
    message, force = sender.messages[-1]
    assert message["mock_hug"]["object_revision"] == 2
    assert message["mock_hug"]["target_boot_id"] == "boot-a"
    assert force is True


def test_motor_reboot_service_requires_physical_support_confirmation():
    sender = Sender()
    client = ControlClient(cfg=SimMappingConfig())
    client.attach_sender(sender)
    service = ControlService(PanelState(), client=client)

    with pytest.raises(RuntimeError, match="physically support the arm"):
        service.reboot_faulted_motor()
    assert sender.messages == []

    service.reboot_faulted_motor(arm_supported=True)

    assert sender.messages == [({"t": "reboot_faulted_motor"}, True)]


def test_clear_safety_fault_sends_recovery_without_enabling_torque():
    sender = Sender()
    client = ControlClient(cfg=SimMappingConfig())
    client.attach_sender(sender)
    service = ControlService(PanelState(), client=client)

    service.clear_safety_fault()

    assert sender.messages == [({"t": "clear_fault"}, True)]


def test_one_axis_command_keeps_previous_goals_after_measured_sag():
    from elesim_protocol import ControlU, control_u_to_sim_q

    mapping = SimMappingConfig()
    sender = Sender()
    client = ControlClient(cfg=mapping)
    client.attach_sender(sender)
    service = ControlService(PanelState(), client=client, mapping_cfg=mapping)
    client.send_target_values(linear_m=-0.05, roll_rad=0.0, theta1_rad=0.1, theta2_rad=-0.1)
    client.accept_telemetry({"q": [-0.06, -0.01, 0.08, -0.08], "torque_enabled": True})
    service.refresh_host_state()
    service.apply_partial_control_u({"roll": 200.0}, source="slider")
    expected_roll = control_u_to_sim_q(ControlU(0, 200, 180, 180), mapping).roll_rad
    assert sender.messages[-1][0]["q"] == pytest.approx([-0.05, expected_roll, 0.1, -0.1])
    assert client.get_state().q.linear_m == pytest.approx(-0.06)


@pytest.mark.parametrize("change", ["target", "disconnect", "torque-off", "error", "negative-ack", "estop"])
def test_goal_cache_is_discarded_when_authority_or_torque_changes(change):
    client = ControlClient()
    client.attach_sender(Sender())
    client.send_target_values(linear_m=-0.05, roll_rad=0, theta1_rad=0, theta2_rad=0)
    assert client.last_commanded_q is not None
    if change == "target":
        client.target_changed("new-target")
    elif change == "disconnect":
        client.peer_connected(False)
    elif change == "torque-off":
        client.accept_telemetry({"torque_enabled": False})
    elif change == "error":
        client.accept_error("rejected")
    elif change == "negative-ack":
        client.accept_ack({"ok": False, "reason": "rejected", "reply_to": "command"})
    else:
        client.estop()
    assert client.last_commanded_q is None
