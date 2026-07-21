from __future__ import annotations

from elesim_ui import control_panel


def test_sim_video_drawer_uses_the_panel_drawer_calling_convention() -> None:
    calls: list[object] = []
    panel = object.__new__(control_panel.ControlPanel)
    panel._draw_sim_video = lambda: calls.append(panel)

    control_panel.draw_sim_video_panel(panel)

    assert calls == [panel]
