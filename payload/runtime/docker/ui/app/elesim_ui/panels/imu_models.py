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
    status = panel.service.imu_model_status
    reader = status.get("reader") or {}
    if not reader:
        imgui.text("IMU reader: disabled or Robot telemetry unavailable")
    else:
        imgui.text("IMU serial: " + ("open" if reader.get("connected") else "disconnected"))
        if reader.get("error"):
            imgui.text_wrapped(str(reader["error"]))
        for name in ("IMU1", "IMU2"):
            sample = reader.get("sensors", {}).get(name)
            if not sample:
                imgui.text(f"{name}: waiting for samples")
                continue
            freshness = ("orientation freshness unverified" if sample.get("source_format") == "full-imu-v1"
                         else "legacy: freshness unverified" if sample.get("sequence") is None
                         else "fresh" if sample.get("fresh") else "stale")
            imgui.text(f"{name}: {freshness}, age {sample['received_age_s']:.2f}s")
            if sample.get("source_format") == "full-imu-v1":
                imgui.text("  Reports: " + ("recent" if sample.get("reports_recent") else "stale"))
            rpy = sample["rpy_rad"]
            if rpy is None:
                imgui.text("  Orientation: waiting for a valid quaternion")
            else:
                imgui.text(f"  RPY (rad): {rpy[0]:+.3f}, {rpy[1]:+.3f}, {rpy[2]:+.3f}")
            full = sample.get("full_imu") or {}
            for key in ("gravity", "gyro", "linear_accel", "accel", "mag"):
                if key in full:
                    vector = full[key]
                    imgui.text(f"  {key}: {vector[0]:+.3f}, {vector[1]:+.3f}, {vector[2]:+.3f}")
        imgui.text("Raw sensor frames; correction input is separate")
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
    requested = status.get("requested") or {}
    if requested:
        state = "active" if status.get("active") else "pending"
        if status.get("error"):
            state = f"rejected: {status['error']}"
        imgui.text(f"IMU model: {requested.get('id', '')} ({state})")
    if panel._imu_model_error:
        imgui.text(f"IMU model request failed: {panel._imu_model_error}")
