from __future__ import annotations

from elesim_protocol import ControlU
from elesim_ui import control_panel
from elesim_ui.panels.go2 import _send_go2_velocity, _stop_go2
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


def test_control_slider_draft_survives_delayed_remote_snapshot() -> None:
    sent: list[dict[str, float]] = []
    panel = object.__new__(control_panel.ControlPanel)
    panel._control_u_draft = None
    panel._control_u_dirty = False
    panel._control_u_dragging = False
    panel._control_u_last_change_at = 0.0
    panel._control_u_last_send_at = 0.0
    panel._control_u_pending = {}
    panel._control_u_send_period_s = 1.0 / 30.0
    panel.service = type(
        "Service",
        (),
        {"apply_partial_control_u": lambda _self, value: sent.append(dict(value))},
    )()

    assert panel.sync_control_u_draft(ControlU(10.0, 20.0, 30.0, 40.0))["roll"] == 20.0
    panel.update_control_u_draft({"roll": 55.0})

    # The snapshot still contains the old value, but the thumb must remain at
    # the locally dragged value until the asynchronous command catches up.
    assert panel.sync_control_u_draft(ControlU(10.0, 20.0, 30.0, 40.0))["roll"] == 55.0
    panel.flush_control_u_draft(force=True)
    assert sent == [{"roll": 55.0}]


def test_go2_teleop_is_rate_limited_and_stop_is_edge_triggered() -> None:
    sent: list[dict[str, float]] = []
    panel = object.__new__(control_panel.ControlPanel)
    panel._go2_send_period_s = 1.0
    panel._go2_last_command_at = -1.0
    panel._go2_was_active = False
    panel._go2_stop_sent = False
    panel.service = type(
        "Service",
        (),
        {"send_go2_velocity": lambda _self, **value: sent.append(dict(value))},
    )()

    assert _send_go2_velocity(panel, vx=0.2, vy=0.0, wz=0.0)
    assert not _send_go2_velocity(panel, vx=0.2, vy=0.0, wz=0.0)
    _stop_go2(panel)
    _stop_go2(panel)

    assert sent == [
        {"vx": 0.2, "vy": 0.0, "wz": 0.0},
        {"vx": 0.0, "vy": 0.0, "wz": 0.0},
    ]
