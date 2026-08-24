from __future__ import annotations

import imgui

from elesim_ui.helpers import panel_header


def _set_result(panel, value) -> None:
    panel._mock_hug_solution = dict(value) if isinstance(value, dict) else {}
    panel._mock_object_message = "hug solution ready"


def _set_error(panel, value) -> None:
    panel._mock_object_message = str(value)


def draw_mock_object_panel(panel) -> None:
    if not panel_header("Mock Object / Hug", visible=True)[0]:
        return
    view = getattr(panel, "_sim_view", None)
    session = None if view is None else view.session
    status = None if session is None else session.snapshot.status
    obj = None if status is None else status.mock_object
    available = () if obj is None else obj.available_assets
    if available and panel._mock_object_asset not in available:
        panel._mock_object_asset = available[0]
    if available:
        imgui.text_disabled("catalog: " + ", ".join(available))

    changed, asset = imgui.input_text(
        "Asset##mock-object-asset", panel._mock_object_asset, 64
    )
    if changed:
        panel._mock_object_asset = str(asset)
    for index, axis in enumerate("XYZ"):
        changed, value = imgui.input_float(
            f"{axis}##mock-object-pos-{axis}", float(panel._mock_object_position[index]), 0.0, 0.0, format="%.3f"
        )
        if changed:
            values = list(panel._mock_object_position)
            values[index] = float(value)
            panel._mock_object_position = values
    for index, axis in enumerate(("Roll", "Pitch", "Yaw")):
        changed, value = imgui.input_float(
            f"{axis} deg##mock-object-euler-{axis}",
            float(panel._mock_object_euler_deg[index]),
            0.0,
            0.0,
            format="%.1f",
        )
        if changed:
            values = list(panel._mock_object_euler_deg)
            values[index] = float(value)
            panel._mock_object_euler_deg = values

    connected = session is not None and obj is not None
    if not connected:
        imgui.text_disabled("Sim session / mock catalog unavailable")
    if imgui.button("Spawn##mock-object-spawn") and connected:
        session.send_command(
            "spawn_mock_object",
            {
                "asset_id": panel._mock_object_asset,
                "position": list(panel._mock_object_position),
                "euler_deg": list(panel._mock_object_euler_deg),
            },
        )
        panel._mock_hug_solution = {}
    imgui.same_line()
    if imgui.button("Remove##mock-object-remove") and connected:
        session.send_command("remove_mock_object")
        panel._mock_hug_solution = {}

    active = connected and obj.state != "empty"
    if imgui.button("Compute hug##mock-hug-compute") and active:
        panel.service.compute_mock_hug(
            on_result=lambda value: _set_result(panel, value),
            on_error=lambda error: _set_error(panel, error),
            request_timeout_s=5.0,
        )
        panel._mock_object_message = "computing..."
    imgui.same_line()
    solution_id = str(panel._mock_hug_solution.get("solution_id", ""))
    if imgui.button("Execute##mock-hug-execute") and solution_id:
        panel.service.execute_mock_hug(
            solution_id,
            on_result=lambda value: _set_error(panel, "executing"),
            on_error=lambda error: _set_error(panel, error),
            request_timeout_s=5.0,
        )
    imgui.same_line()
    if imgui.button("Detach##mock-hug-detach") and active:
        session.send_command("detach_mock_object")

    if obj is not None:
        imgui.text_disabled(
            f"state={obj.state} revision={obj.revision} asset={obj.asset_id or '-'}"
        )
        if obj.reason:
            imgui.text_wrapped(obj.reason)
    if panel._mock_object_message:
        imgui.text_wrapped(panel._mock_object_message)


__all__ = ["draw_mock_object_panel"]
