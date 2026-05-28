from __future__ import annotations

import imgui


def _browse_sag_model_path(initial_path: str) -> str | None:
    try:
        import os
        import tkinter as tk
        from tkinter import filedialog

        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        default_assets_dir = os.path.join(project_root, "assets")
        root = tk.Tk()
        root.withdraw()
        root.update_idletasks()
        initial = str(initial_path or "").strip()
        initial_dir = os.path.dirname(initial) if initial else default_assets_dir
        if not os.path.isdir(initial_dir):
            initial_dir = default_assets_dir if os.path.isdir(default_assets_dir) else os.getcwd()
        selected = filedialog.askopenfilename(
            title="Select sag model JSON",
            initialdir=initial_dir,
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        root.destroy()
        selected = str(selected or "").strip()
        return selected or None
    except Exception:
        return None


def draw_sag_panel(panel) -> None:
    if not panel._sag_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._sag_header_init_open = True
    if imgui.collapsing_header("Sag Model", visible=True)[0]:
        changed, sag_path = imgui.input_text("sag model path", panel._sag_model_path_draft, 512)
        if changed:
            panel._sag_model_path_draft = str(sag_path)
        if imgui.button("Browse"):
            selected = _browse_sag_model_path(panel._sag_model_path_draft)
            if selected:
                panel._sag_model_path_draft = str(selected)
        imgui.same_line()
        if imgui.button("Load Model"):
            try:
                resolved_path, model = panel.service.load_sag_model(panel._sag_model_path_draft)
                panel._sag_model_path_draft = str(resolved_path)
                panel.service.send_current_target(source="target")
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

        model_path = str(panel.state.sag_model_path or "").strip()
        if panel._sag_status_text and (not panel._sag_status_ok):
            imgui.text_colored(panel._sag_status_text, 1.0, 0.35, 0.35)
        elif model_path:
            imgui.text_wrapped(f"Model loaded: {model_path}")
        else:
            imgui.text("Model not loaded")
