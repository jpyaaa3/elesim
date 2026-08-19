from __future__ import annotations

from elesim_pilot.pick import ControlService, PanelState


def test_pick_stop_does_not_stop_the_camera_or_perception_capture() -> None:
    service = ControlService(PanelState())
    calls: list[str] = []
    service.send_go2_velocity = lambda **_kwargs: calls.append("go2_stop")
    service.stop_gaze_stabilizer = lambda: calls.append("gaze_stop")
    service.stop_object_pick = lambda: calls.append("pick_stop")
    service.stop_perception_capture = lambda **_kwargs: calls.append("camera_stop")

    service.stop_pick_e2e()

    assert calls == ["go2_stop", "gaze_stop", "pick_stop"]
