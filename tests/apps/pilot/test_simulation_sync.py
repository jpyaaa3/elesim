from __future__ import annotations

from elesim_protocol import SimulationStatusPayload
from elesim_pilot.simulation_sync import SimulationWorkflowSync, cancellation_reason


def status(*, epoch: int = 0, paused: bool = False) -> SimulationStatusPayload:
    return SimulationStatusPayload(
        epoch=epoch,
        paused=paused,
        speed=1.0,
        debug_visible=True,
        sim_time_s=0.0,
    )


class Service:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def stop_pick_e2e(self) -> None:
        self.calls.append("pick")

    def stop_gaze_stabilizer(self) -> None:
        self.calls.append("gaze")


def test_transition_only_cancels_on_pause_edge_or_epoch_change() -> None:
    running = status(epoch=4, paused=False)
    assert cancellation_reason(None, running) == ""
    assert cancellation_reason(running, status(epoch=4, paused=True)) == "simulation paused"
    assert "epoch changed" in cancellation_reason(running, status(epoch=5, paused=False))
    assert cancellation_reason(status(epoch=4, paused=True), status(epoch=4, paused=True)) == ""


def test_cancellation_runs_outside_the_status_accept_path() -> None:
    service = Service()
    sync = SimulationWorkflowSync(service, autostart=False)

    sync.accept(status(epoch=0, paused=True))
    assert service.calls == []

    assert sync.process_one() is True
    assert service.calls == ["pick", "gaze"]


def test_pause_and_target_change_cancel_registered_workflows_and_clear_status() -> None:
    service = Service()
    sync = SimulationWorkflowSync(service, autostart=False)
    reasons: list[str] = []
    sync.add_cancel_callback(reasons.append)
    sync.accept(status(epoch=1, paused=False))

    sync.accept(status(epoch=1, paused=True))
    assert sync.process_one() is True
    assert reasons == ["simulation paused"]

    sync.clear("motion target changed")
    assert sync.latest is None
    assert reasons[-1] == "motion target changed"
