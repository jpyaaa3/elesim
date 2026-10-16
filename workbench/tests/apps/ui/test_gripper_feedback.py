from types import SimpleNamespace

import pytest

from elesim_ui.panels import control_4dof


@pytest.mark.parametrize("connected,expected", [(True, True), (False, False)])
def test_gripper_switch_prefers_live_measured_state_over_requested_state(monkeypatch, connected, expected):
    seen = []
    panel = SimpleNamespace(state=SimpleNamespace(claw_closed=False),
                            _host_state=SimpleNamespace(connected=connected, claw_closed=True))
    monkeypatch.setattr(control_4dof, "_control_label", lambda *a: None)
    monkeypatch.setattr(control_4dof, "toggle_switch", lambda p, key, value, **kw: seen.append(value))
    monkeypatch.setattr(control_4dof, "_warn_button", lambda *a: False)
    monkeypatch.setattr(control_4dof.imgui, "same_line", lambda: None)
    control_4dof._draw_gripper_row(panel)
    assert seen == [expected]
    assert panel.state.claw_closed is False  # Feedback does not create a command.
