from types import SimpleNamespace as NS

import pytest

from elesim_ui.panels import floor_calibration


@pytest.mark.parametrize("stable,expected", [(1, []), (3, [("confirm_floor_calibration", "plane-7")])])
def test_only_stable_plane_can_be_confirmed_and_no_motion_is_requested(monkeypatch, stable, expected):
    calls, buttons, messages = [], [], []
    monkeypatch.setattr(floor_calibration, "panel_header", lambda *_a, **_k: (True,))
    def button(label):
        buttons.append(label)
        return label == "Confirm horizontal reference"
    monkeypatch.setattr(floor_calibration, "imgui", NS(text=messages.append,
        text_wrapped=messages.append, combo=lambda *_a: (False, 0),
        button=button, same_line=lambda: None))
    status = {"enabled": True, "state": "candidate", "synchronization_verified": False,
        "required_stable_frames": 3, "candidates": [{"id": "plane-7", "stable_frames": stable,
            "height_above_plane_m": 0.8, "area_m2": 1.2}]}
    service = NS(floor_calibration_status=status,
        floor_calibration_async=lambda name, *args, **kwargs: calls.append((name, *args)))
    floor_calibration.draw_floor_calibration_panel(NS(service=service))
    assert calls == expected
    assert any("timing unverified" in text for text in messages)
    assert any("Plane direction" in text for text in messages)
    assert not any("Height error" in text or "configured height" in text for text in messages)


def test_export_path_is_taken_from_async_result_on_pilot_not_invented_locally(monkeypatch):
    monkeypatch.setattr(floor_calibration, "panel_header", lambda *_a, **_k: (True,))
    monkeypatch.setattr(floor_calibration, "imgui", NS(text=lambda *_: None,
        text_wrapped=lambda *_: None, button=lambda label: label == "Export plane observations",
        same_line=lambda: None))
    def submit(name, *args, **kwargs):
        assert name == "export_floor_calibration"
        kwargs["on_result"]("/tmp/elesim-floor-calibration-example/observations.jsonl")
    panel = NS(service=NS(floor_calibration_status={"enabled": True, "state": "lost"},
                          floor_calibration_async=submit))
    floor_calibration.draw_floor_calibration_panel(panel)
    assert panel._floor_export_path == "/tmp/elesim-floor-calibration-example/observations.jsonl"


def test_stopped_perception_keeps_export_available(monkeypatch):
    messages = []
    monkeypatch.setattr(floor_calibration, "panel_header", lambda *_a, **_k: (True,))
    monkeypatch.setattr(floor_calibration, "imgui", NS(text=messages.append,
        text_wrapped=messages.append, button=lambda label: label == "Export plane observations"))
    def submit(name, *args, **kwargs):
        assert name == "export_floor_calibration"
        kwargs["on_result"]("/tmp/retained-observations.jsonl")
    panel = NS(service=NS(floor_calibration_status={"enabled": False, "state": "stopped", "records": 20},
                         floor_calibration_async=submit))
    floor_calibration.draw_floor_calibration_panel(panel)
    assert panel._floor_export_path == "/tmp/retained-observations.jsonl"
    assert any("/tmp/retained-observations.jsonl" in text for text in messages)
