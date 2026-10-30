from __future__ import annotations

import pytest

from elesim_protocol import SimulationCommandRequest
from elesim_sim.simulation.operator_control import (
    SimulationOperatorCommand,
    SimulationOperatorController,
    SimulationOperatorMailbox,
)


def command(name: str, arguments: dict[str, object], request_id: str = "request-1"):
    request = SimulationCommandRequest.from_payload(
        {
            "schema_version": 1,
            "request_id": request_id,
            "session_id": "session-a",
            "command": name,
            "arguments": arguments,
        }
    )
    return SimulationOperatorCommand.from_request(request, ui_id="ui-a")


def test_scene_edit_pauses_and_advances_epoch_before_callback_and_retains_pause_on_failure():
    seen = []
    controller = SimulationOperatorController(reset_environment=lambda: None,
        observer_command=lambda *a: None,
        scene_command=lambda cmd: seen.append((controller.paused, controller.epoch)) or "ready")
    assert controller.apply(command('load_scene', {'manifest': 'scene.json'})) == (True, 'ready')
    assert seen == [(True, 1)] and controller.paused is False
    def fail(cmd): raise RuntimeError('camera failed')
    controller.scene_command = fail
    with pytest.raises(RuntimeError, match='camera failed'):
        controller.apply(command('clear_scene', {}))
    assert controller.paused and controller.epoch == 2


def test_revoked_session_invalidates_commands_already_in_the_mailbox():
    mailbox = SimulationOperatorMailbox()
    mailbox.set_session('ui-a', 'session-a')
    item = command('load_scene', {'manifest': 'scene.json'})
    mailbox.enqueue(item)
    mailbox.set_session('ui-b', 'session-b')
    queued = mailbox.drain()[0]
    assert not mailbox.owns(queued) and not mailbox.is_current(queued)


def test_scene_failure_reason_remains_serializable_with_a_long_genesis_error():
    from elesim_protocol import SimulationResultPayload

    mailbox = SimulationOperatorMailbox()
    mailbox.complete(command('clear_scene', {}), ok=False, reason='failed: ' + 'x' * 2000)
    result = mailbox.take_results()[0].payload
    assert len(result.reason) == 512
    assert SimulationResultPayload.from_payload(result.to_payload()).ok is False


def test_mailbox_coalesces_adjacent_camera_motion_and_keeps_results() -> None:
    mailbox = SimulationOperatorMailbox(max_pending=4)
    assert mailbox.enqueue(command("orbit", {"dx": 0.1, "dy": 0.2}, "orbit-1"))
    assert mailbox.enqueue(command("orbit", {"dx": -0.04, "dy": 0.1}, "orbit-2"))

    pending = mailbox.drain()
    assert len(pending) == 1
    assert pending[0].arguments["dx"] == pytest.approx(0.06)
    assert pending[0].arguments["dy"] == pytest.approx(0.3)
    assert pending[0].request_ids == ("orbit-1", "orbit-2")
    mailbox.complete(pending[0], ok=True, reason="applied")
    assert [result.request_id for result in mailbox.take_results()] == ["orbit-1", "orbit-2"]


def test_mailbox_is_bounded_without_blocking_the_dds_thread() -> None:
    mailbox = SimulationOperatorMailbox(max_pending=1)
    assert mailbox.enqueue(command("pause", {})) is True
    assert mailbox.enqueue(command("reset", {}, "request-2")) is False


def test_mailbox_bounds_results_and_supports_incremental_flush() -> None:
    mailbox = SimulationOperatorMailbox(max_pending=4, max_results=2)
    for index in range(4):
        mailbox.complete(
            command("pause", {}, f"request-{index}"),
            ok=True,
            reason="paused",
        )

    assert [item.request_id for item in mailbox.take_results(max_items=1)] == [
        "request-2"
    ]
    assert [item.request_id for item in mailbox.take_results()] == ["request-3"]


def test_runtime_controller_preserves_pause_across_reset_and_single_steps() -> None:
    resets: list[str] = []
    observer_commands: list[tuple[str, dict[str, object]]] = []
    cancellations: list[str] = []
    controller = SimulationOperatorController(
        reset_environment=lambda: resets.append("reset"),
        observer_command=lambda name, args: observer_commands.append((name, args)),
        mock_hug_cancel=cancellations.append,
    )

    assert controller.apply(command("pause", {})) == (True, "paused")
    assert cancellations == ["simulation paused"]
    assert controller.apply(command("step", {"count": 2})) == (True, "step queued")
    assert controller.should_step() is True
    assert controller.should_step() is True
    assert controller.should_step() is False
    assert controller.apply(command("reset", {})) == (True, "reset")
    assert resets == ["reset"]
    assert controller.paused is True
    assert controller.epoch == 1


def test_step_is_rejected_while_running_and_camera_commands_mark_observer_dirty() -> None:
    observer_commands: list[tuple[str, dict[str, object]]] = []
    controller = SimulationOperatorController(
        reset_environment=lambda: None,
        observer_command=lambda name, args: observer_commands.append((name, args)),
    )

    assert controller.apply(command("step", {"count": 1})) == (
        False,
        "single-step requires a paused simulation",
    )
    assert controller.apply(command("zoom", {"delta": 0.1})) == (True, "view updated")
    assert controller.take_observer_dirty() is True
    assert controller.take_observer_dirty() is False
    assert observer_commands == [("zoom", {"delta": 0.1})]


def test_failed_environment_reset_does_not_advance_epoch() -> None:
    def fail_reset() -> None:
        raise RuntimeError("Genesis scene reset failed")

    controller = SimulationOperatorController(
        reset_environment=fail_reset,
        observer_command=lambda _name, _args: None,
    )

    with pytest.raises(RuntimeError, match="Genesis scene reset failed"):
        controller.apply(command("reset", {}))

    assert controller.epoch == 0


def test_speed_and_debug_visibility_are_part_of_status() -> None:
    controller = SimulationOperatorController(
        reset_environment=lambda: None,
        observer_command=lambda _name, _args: None,
    )
    controller.apply(command("set_speed", {"scale": 0.5}))
    controller.apply(command("set_debug_visible", {"visible": False}))

    status = controller.status(sim_time_s=2.5)
    assert status.speed == 0.5
    assert status.debug_visible is False
    assert status.sim_time_s == 2.5


def test_mock_object_commands_and_status_use_genesis_thread_callbacks() -> None:
    applied: list[tuple[str, dict[str, object]]] = []
    controller = SimulationOperatorController(
        reset_environment=lambda: None,
        observer_command=lambda _name, _args: None,
        mock_object_command=lambda name, args: applied.append((name, args)),
        mock_object_status=lambda: {
            "available_assets": ("demo_box",),
            "state": "empty",
            "revision": 0,
        },
    )
    spawn = command(
        "spawn_mock_object",
        {"asset_id": "demo_box", "position": [0.5, 0.0, 0.4], "euler_deg": [0.0, 0.0, 0.0]},
    )
    assert controller.apply(spawn) == (True, "mock object updated")
    assert applied[0][0] == "spawn_mock_object"
    assert controller.status(sim_time_s=0.0).mock_object.available_assets == ("demo_box",)
