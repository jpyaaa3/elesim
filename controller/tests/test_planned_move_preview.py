from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from elesim_controller.pick import ControlService, PanelState
from elesim_controller.pick.control_ownership import ControlOwner

CONFIG_PATH = Path(__file__).parents[1] / "config" / "config.yaml"

_WAYPOINTS = [np.array([0.0, 0.0, 0.0, 0.0]), np.array([-0.1, 0.2, 0.3, -0.1])]


class _RecordingClient:
    def __init__(self) -> None:
        self.calls: list[list[tuple[float, ...]]] = []

    def send_planned_move_preview(self, waypoints, *, source: str = "target") -> None:
        self.calls.append([tuple(float(x) for x in wp) for wp in waypoints])


def _planned_service(client: _RecordingClient, *, waypoints=None) -> ControlService:
    service = ControlService(PanelState(), client=client, config_path=str(CONFIG_PATH))
    if waypoints is not None:
        with service._planned_move._lock:
            service._planned_move._waypoints = list(waypoints)
        service._planned_move._set_status(phase="planned", message="", waypoint_count=len(waypoints) - 1)
    return service


def test_start_planned_move_preview_noop_when_not_planned() -> None:
    client = _RecordingClient()
    service = _planned_service(client)
    assert service._planned_move.status().phase == "idle"

    service.start_planned_move_preview()

    assert client.calls == []


def test_start_planned_move_preview_noop_when_planned_with_no_waypoints() -> None:
    client = _RecordingClient()
    service = _planned_service(client, waypoints=[])
    service._planned_move._set_status(phase="planned", message="", waypoint_count=0)

    service.start_planned_move_preview()

    assert client.calls == []


def test_start_planned_move_preview_sends_the_generated_waypoints() -> None:
    client = _RecordingClient()
    service = _planned_service(client, waypoints=_WAYPOINTS)

    service.start_planned_move_preview()

    assert len(client.calls) == 1
    sent = client.calls[0]
    assert len(sent) == len(_WAYPOINTS)
    for got, expected in zip(sent, _WAYPOINTS):
        assert got == pytest.approx(tuple(float(x) for x in expected))


def test_start_planned_move_preview_does_not_touch_control_ownership() -> None:
    """Preview is a pure Simulator-side visual -- it must work (and must not
    disturb who currently holds the arm) even while another owner holds it,
    unlike ``start_planned_move_execute`` which requires PLANNED_MOVE ownership."""
    client = _RecordingClient()
    service = _planned_service(client, waypoints=_WAYPOINTS)
    service._planned_move._ownership.acquire(ControlOwner.GAZE_TRACK)

    service.start_planned_move_preview()

    assert len(client.calls) == 1
    assert service._planned_move._ownership.owner == ControlOwner.GAZE_TRACK
