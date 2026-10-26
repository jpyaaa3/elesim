from __future__ import annotations

from types import SimpleNamespace

import glfw
import pytest

from elesim_ui import control_panel
from elesim_ui.panels import control_4dof


@pytest.fixture
def keyboard(monkeypatch):
    panel = object.__new__(control_panel.ControlPanel)
    resets = []
    panel.service = SimpleNamespace(reset_simulation=lambda: resets.append("reset"))
    panel._go2_was_active = True
    panel._go2_respawn_blocked = False
    panel._clear_respawn_shortcut()
    clock = [10.]
    callbacks, focus_callbacks, renderers, forwarded = {}, {}, {}, []
    monkeypatch.setattr(control_panel.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(glfw, "get_window_attrib", lambda *_: glfw.TRUE)
    monkeypatch.setattr(glfw, "set_key_callback", lambda window, callback: callbacks.update({window: callback}))
    monkeypatch.setattr(glfw, "set_window_focus_callback", lambda window, callback: focus_callbacks.update({window: callback}))
    for window in ("main", "camera"):
        renderer = SimpleNamespace(io=SimpleNamespace(want_text_input=False),
                                   keyboard_callback=lambda *event: forwarded.append(event))
        renderers[window] = renderer
        panel._install_respawn_shortcut(window, renderer)

    def event(at, key=glfw.KEY_R, action=glfw.PRESS, mods=0, window="main"):
        clock[0] = at
        callbacks[window](window, key, 0, action, mods)

    return SimpleNamespace(panel=panel, resets=resets, event=event,
                           focus=focus_callbacks, renderers=renderers, forwarded=forwarded)


@pytest.mark.parametrize("window", ("main", "camera"))
def test_two_r_presses_respawn_once_and_block_held_motion(keyboard, window):
    keyboard.event(10., window=window)
    assert keyboard.resets == []
    keyboard.event(10.05, action=glfw.RELEASE, window=window)
    keyboard.event(10.4, window=window)
    assert keyboard.resets == ["reset"]
    assert keyboard.panel._go2_respawn_blocked is True
    assert keyboard.panel._go2_was_active is False
    keyboard.event(10.45, action=glfw.RELEASE, window=window)
    keyboard.event(10.49, window=window)
    assert keyboard.resets == ["reset"]


def test_holding_r_does_not_turn_key_repeat_into_a_double_press(keyboard):
    keyboard.event(10.)
    keyboard.event(10.1, action=glfw.REPEAT)
    keyboard.event(10.2, action=glfw.REPEAT)
    assert keyboard.resets == []
    keyboard.event(10.3, action=glfw.RELEASE)
    keyboard.event(10.4)
    assert keyboard.resets == ["reset"]


def test_a_late_second_r_starts_a_new_pair(keyboard):
    keyboard.event(10.)
    keyboard.event(10.6)
    assert keyboard.resets == []
    keyboard.event(10.8)
    assert keyboard.resets == ["reset"]


def test_another_key_between_r_presses_cancels_the_pair(keyboard):
    keyboard.event(10.)
    keyboard.event(10.1, key=glfw.KEY_A)
    keyboard.event(10.2)
    assert keyboard.resets == []
    keyboard.event(10.3)
    assert keyboard.resets == ["reset"]


@pytest.mark.parametrize("mods", (glfw.MOD_CONTROL, glfw.MOD_ALT, glfw.MOD_SUPER, glfw.MOD_SHIFT))
def test_modified_r_does_not_respawn_or_complete_an_unmodified_pair(keyboard, mods):
    keyboard.event(10.)
    keyboard.event(10.1, mods=mods)
    keyboard.event(10.2, mods=mods)
    keyboard.event(10.3)
    assert keyboard.resets == []


def test_typing_r_in_an_input_cancels_the_pair(keyboard):
    keyboard.event(10.)
    keyboard.renderers["main"].io.want_text_input = True
    keyboard.event(10.1)
    keyboard.event(10.2)
    keyboard.renderers["main"].io.want_text_input = False
    keyboard.event(10.3)
    assert keyboard.resets == []


def test_leaving_the_window_cancels_the_pair(keyboard):
    keyboard.event(10.)
    keyboard.focus["main"]("main", glfw.FALSE)
    keyboard.focus["main"]("main", glfw.TRUE)
    keyboard.event(10.2)
    assert keyboard.resets == []


def test_unfocused_window_cannot_trigger_respawn(keyboard, monkeypatch):
    monkeypatch.setattr(glfw, "get_window_attrib", lambda *_: glfw.FALSE)
    keyboard.event(10.)
    keyboard.event(10.2)
    assert keyboard.resets == []


def test_r_presses_in_different_windows_do_not_form_a_pair(keyboard):
    keyboard.event(10., window="main")
    keyboard.event(10.1, window="camera")
    assert keyboard.resets == []


def test_shortcut_keeps_forwarding_keyboard_events_to_imgui(keyboard):
    keyboard.event(10.)
    keyboard.event(10.1, action=glfw.REPEAT)
    keyboard.event(10.2, action=glfw.RELEASE)
    assert keyboard.forwarded == [
        ("main", glfw.KEY_R, 0, glfw.PRESS, 0),
        ("main", glfw.KEY_R, 0, glfw.REPEAT, 0),
        ("main", glfw.KEY_R, 0, glfw.RELEASE, 0),
    ]


def test_respawn_button_uses_the_same_reset_and_motion_block(keyboard, monkeypatch):
    monkeypatch.setattr(control_4dof, "_control_label", lambda *_: None)
    monkeypatch.setattr(control_4dof, "scaled", lambda _panel, value: value)
    monkeypatch.setattr(control_4dof, "imgui", SimpleNamespace(button=lambda *_: True))
    control_4dof._draw_respawn_row(keyboard.panel)
    assert keyboard.resets == ["reset"]
    assert keyboard.panel._go2_respawn_blocked is True
    assert keyboard.panel._go2_was_active is False
