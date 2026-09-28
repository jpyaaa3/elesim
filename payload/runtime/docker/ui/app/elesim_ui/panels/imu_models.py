"""Pilot-owned IMU model selector (legacy panel slot)."""

from __future__ import annotations

import imgui

from elesim_ui.helpers import panel_header


def draw_imu_model_panel(panel) -> None:
    if not panel._imu_model_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._imu_model_header_init_open = True
    if not panel_header("IMU Model", visible=True)[0]:
        return
    models = panel.service.imu_models
    if not models:
        imgui.text("No Pilot IMU models available")
        return
    labels = [str(model.get("label", model.get("id", ""))) for model in models]
    panel._imu_model_index = min(panel._imu_model_index, len(models) - 1)
    changed, selected = imgui.combo(
        "IMU interpretation model", panel._imu_model_index, labels
    )
    if changed:
        panel._imu_model_index = selected
    if imgui.button("Apply IMU Model"):
        panel._imu_model_error = ""
        panel.service.select_imu_model_async(
            str(models[panel._imu_model_index]["id"]),
            on_result=lambda _result: None,
            on_error=lambda error: setattr(panel, "_imu_model_error", error),
        )
    status = panel.service.imu_model_status
    requested = status.get("requested") or {}
    if requested:
        state = "active" if status.get("active") else "pending"
        if status.get("error"):
            state = f"rejected: {status['error']}"
        imgui.text(f"IMU model: {requested.get('id', '')} ({state})")
    if panel._imu_model_error:
        imgui.text(f"IMU model request failed: {panel._imu_model_error}")
