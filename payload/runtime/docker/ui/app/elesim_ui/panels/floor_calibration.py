"""Read-only plane evidence and explicit reference selection owned by Pilot."""
from __future__ import annotations

import imgui

from elesim_ui.helpers import panel_header


def draw_floor_calibration_panel(panel):
    status = getattr(panel.service, "floor_calibration_status", {})
    if not status:
        return
    if not panel_header("Visual Calibration", visible=True)[0]:
        return
    imgui.text("Plane direction: " + str(status.get("state", "idle")))
    if status.get("reason"):
        imgui.text_wrapped(str(status["reason"]))
    if not status.get("enabled"):
        imgui.text_wrapped("Start perception to observe depth planes.")
        if status.get("records", 0) and imgui.button("Export plane observations"):
            _submit(panel, "export_floor_calibration")
        _draw_result(panel)
        return
    if status.get("error"):
        imgui.text_wrapped(str(status["error"]))
    if not status.get("synchronization_verified"):
        imgui.text_wrapped("Capture/pose timing unverified; correction unavailable.")
    angle = status.get("normal_error_deg")
    if angle is not None:
        imgui.text(f"Tilt error: {float(angle):.2f} deg")
    reference = status.get("reference")
    if reference:
        imgui.text("Reference: " + ("declared plane direction" if reference.get("kind") in {"configured", "configured_direction"} else "confirmed horizontal surface"))
    candidates = status.get("candidates", [])
    if candidates:
        labels = [f"{p['id']} | {p['area_m2']:.2f} m2"
                  for p in candidates]
        selected_id = getattr(panel, "_floor_candidate_id", "")
        index = next((i for i, p in enumerate(candidates) if p["id"] == selected_id), 0)
        _, index = imgui.combo("Reference plane", index, labels)
        panel._floor_candidate_id = candidates[index]["id"]
        required = int(status.get("required_stable_frames", 3))
        if candidates[index].get("stable_frames", 0) >= required:
            if imgui.button("Confirm horizontal reference"):
                _submit(panel, "confirm_floor_calibration", panel._floor_candidate_id)
        else:
            imgui.text(f"Waiting for stable plane: {candidates[index].get('stable_frames', 0)}/{required}")
    if imgui.button("Reset plane reference"):
        _submit(panel, "reset_floor_calibration")
    imgui.same_line()
    if imgui.button("Export plane observations"):
        _submit(panel, "export_floor_calibration")
    if status.get("correction_available") or status.get("apply_requested"):
        changed, value = imgui.checkbox("Apply visual tilt correction", bool(status.get("apply_requested")))
        if changed:
            panel.service.update_perception_config({"floor_calibration_apply": value})
    imgui.text(f"Observations: {status.get('processed', 0)}, {status.get('processing_ms', 0):.1f} ms")
    _draw_result(panel)


def _draw_result(panel):
    if getattr(panel, "_floor_export_path", ""):
        imgui.text_wrapped("Pilot file: " + panel._floor_export_path)
    if getattr(panel, "_floor_error", ""):
        imgui.text_wrapped(panel._floor_error)


def _submit(panel, name, *args):
    panel._floor_error = ""
    panel.service.floor_calibration_async(name, *args,
        on_result=lambda result: setattr(panel, "_floor_export_path", result)
            if name == "export_floor_calibration" else None,
        on_error=lambda error: setattr(panel, "_floor_error", str(error)))
