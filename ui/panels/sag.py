from __future__ import annotations

from pathlib import Path

import imgui

from ui.helpers import panel_header
from ui.file_dialog import browse_open_file_path


def sag_browse_initial_dir(initial_path: str) -> str:
    project_root = Path(__file__).resolve().parents[2]
    default_dir = project_root / "configs" / "sag"
    initial = str(initial_path or "").strip()
    initial_path_obj = Path(initial).expanduser() if initial else default_dir
    if not initial_path_obj.is_absolute():
        initial_path_obj = project_root / initial_path_obj
    initial_dir = initial_path_obj.parent if initial_path_obj.suffix else initial_path_obj
    if not initial_dir.is_dir():
        initial_dir = default_dir if default_dir.is_dir() else project_root
    return str(initial_dir)


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
