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


def test_workflow_commands_cannot_be_delegated_to_robot_or_sim() -> None:
    client = ControlClient(cfg=SimMappingConfig())
    with pytest.raises(RuntimeError, match="pilot deployment"):
        client.send_mobile_pick_start()


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
