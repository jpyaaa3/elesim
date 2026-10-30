"""Import a staged scene bundle into the selected Sim session."""
import imgui
from elesim_ui.helpers import panel_header


def draw_scene_panel(panel):
    if not panel_header("Sim Scene", visible=True)[0]:
        return
    view = getattr(panel, "_sim_view", None)
    session = getattr(view, "session", None)
    if session is None or not session.supports_scene_rebuild:
        imgui.text_disabled("Runtime scene loading is unavailable on the selected Sim")
        return
    _, reference = imgui.input_text("Scene bundle##scene-manifest",
        getattr(panel, "_scene_manifest_ref", ""), 513)
    panel._scene_manifest_ref = reference
    imgui.text_wrapped("Bundle path relative to the Sim import directory, e.g. scan-1/scene.json")
    pending = session.snapshot.pending_commands > 0
    if imgui.button("Load scene##load-scene") and not pending:
        session.send_command("load_scene", {"manifest": reference})
    imgui.same_line()
    if imgui.button("Clear scene##clear-scene") and not pending:
        session.send_command("clear_scene")
    if pending:
        imgui.text_disabled("Waiting for Sim...")
    result = session.snapshot.last_result
    if result is not None and result.command in {"load_scene", "clear_scene"}:
        imgui.text_wrapped(result.reason)
