from types import SimpleNamespace as NS

import pytest

from elesim_ui.panels import ik


@pytest.mark.parametrize("phase,clicked,expected", [
    ("idle", "Plan XYZ", "plan_arm_to_target"),
    ("ready", "Execute path", "execute_arm_trajectory"),
    ("planning", "Execute path", None),
    ("executing", "Plan XYZ", None),
    ("executing", "Cancel path", "cancel_arm_trajectory"),
])
def test_plan_execute_cancel_are_separate_and_gated(monkeypatch, phase, clicked, expected):
    disabled, calls = [False], []
    def begin(value):
        previous, disabled[0] = disabled[0], value
        return previous
    monkeypatch.setattr(ik, "begin_disabled_ui", begin)
    monkeypatch.setattr(ik, "end_disabled_ui", lambda previous: disabled.__setitem__(0, previous))
    no_op = lambda *args, **kwargs: None
    monkeypatch.setattr(ik, "imgui", NS(separator=no_op, text=no_op, text_disabled=no_op,
        text_wrapped=no_op, same_line=no_op,
        button=lambda name: name == clicked and not disabled[0]))
    status = dict(available=True, phase=phase, plan_id="prepared-id", duration_s=1.,
                  current_xyz=[.3, 0., .2], target_xyz=[.3, 0., .21])
    service = NS(session=NS(service_value=lambda *args: status))
    for name in ("plan_arm_to_target", "execute_arm_trajectory", "cancel_arm_trajectory"):
        setattr(service, name, lambda *args, method=name, **kwargs: calls.append((method, args)))
    panel = NS(service=service, state=NS(set_target=no_op))
    ik._draw_curobo(panel)
    assert [name for name, args in calls] == ([] if expected is None else [expected])
    if expected == "execute_arm_trajectory":
        assert calls[0][1] == ("prepared-id",)


def test_request_error_is_kept_after_callback_and_other_snapshots():
    callbacks = {}
    def request(**kwargs): callbacks.update(kwargs)
    panel = NS(service=NS(plan_arm_to_target=request))
    ik._path_request(panel, "plan_arm_to_target")
    assert panel._curobo_action["pending"]
    callbacks["on_error"]("Robot not connected")
    assert not panel._curobo_action["pending"]
    assert panel._curobo_action["error"] == "Robot not connected"
