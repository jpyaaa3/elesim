from __future__ import annotations

from elesim_ui import control_panel
from elesim_ui.sim_view import SimViewState


def test_sim_video_drawer_uses_the_panel_drawer_calling_convention() -> None:
    calls: list[object] = []
    panel = object.__new__(control_panel.ControlPanel)
    panel._draw_sim_video = lambda: calls.append(panel)

    control_panel.draw_sim_video_panel(panel)

    assert calls == [panel]


def test_sim_preview_swap_is_explicit_and_reversible() -> None:
    state = SimViewState()

    state.swap_streams()
    assert state.main_stream == "hand_eye_preview"
    assert state.preview_stream == "observer"

    state.swap_streams()
    assert state.main_stream == "observer"
