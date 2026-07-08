from __future__ import annotations

import imgui

from ui.helpers import panel_header
from ui.file_dialog import browse_open_file_path


def sag_browse_initial_dir(initial_path: str) -> str:
    import os

    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    default_assets_dir = os.path.join(project_root, "assets")
    initial = str(initial_path or "").strip()
    initial_dir = os.path.dirname(initial) if initial else default_assets_dir
    if not os.path.isdir(initial_dir):
        initial_dir = default_assets_dir if os.path.isdir(default_assets_dir) else os.getcwd()
    return initial_dir


def browse_sag_model_path(initial_path: str) -> str | None:
    try:
        return browse_open_file_path(
            title="Select sag model JSON",
            initial_dir=sag_browse_initial_dir(initial_path),
            extensions=(".json",),
        )
    except Exception:
        return None


def draw_sag_panel(panel) -> None:
    if not panel._sag_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._sag_header_init_open = True
    if panel_header("Sag Model", visible=True)[0]:
        changed, sag_path = imgui.input_text("sag model path", panel._sag_model_path_draft, 512)
        if changed:
            panel._sag_model_path_draft = str(sag_path)
        if imgui.button("Browse"):
            panel.request_file_browse(kind="sag", initial_path=panel._sag_model_path_draft)
        imgui.same_line()
        if imgui.button("Load Model"):
            try:
                resolved_path, model = panel.service.load_sag_model(panel._sag_model_path_draft)
                panel._sag_model_path_draft = str(resolved_path)
                panel.service.send_sag_model_meta(source="target")
                raw_type = str(model.get("model_type", "") or "").strip()
                if raw_type:
                    model_type = raw_type
                elif any(k in model for k in ("c1_family", "c1_params", "a1", "b1_coeffs", "c2_family", "c2_params", "a2", "b2_coeffs")):
                    model_type = "refined"
                elif any(k in model for k in ("seg1_distribution", "seg1_amplitude", "seg2_distribution", "seg2_amplitude")):
                    model_type = "legacy"
                else:
                    model_type = "unknown"
                panel._sag_status_text = f"loaded: {resolved_path} ({model_type})"
                panel._sag_status_ok = True
            except Exception as exc:
                panel._sag_status_text = f"load failed: {exc}"
                panel._sag_status_ok = False
