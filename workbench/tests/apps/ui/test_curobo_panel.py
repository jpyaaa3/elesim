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


@pytest.mark.parametrize("current_xyz", [
    None,
    [],
    [0.1, 0.2],
    [0.1, 0.2, float("nan")],
    "0.1,0.2,0.3",
])
def test_use_current_xyz_ignores_missing_or_invalid_pose(monkeypatch, current_xyz):
    disabled_messages = []
    targets = []
    no_op = lambda *args, **kwargs: None
    monkeypatch.setattr(ik, "begin_disabled_ui", lambda _disabled: None)
    monkeypatch.setattr(ik, "end_disabled_ui", no_op)
    monkeypatch.setattr(ik, "imgui", NS(
        separator=no_op,
        text=no_op,
        text_disabled=lambda value: disabled_messages.append(str(value)),
        text_wrapped=no_op,
        same_line=no_op,
        # Simulate a backend that reports the click even though the control is
        # drawn in a disabled block; the handler must still reject bad data.
        button=lambda label: label == "Use current XYZ",
    ))
    status = {"available": True, "phase": "idle", "current_xyz": current_xyz}
    panel = NS(
        service=NS(session=NS(service_value=lambda *_args: status)),
        state=NS(set_target=lambda *xyz: targets.append(xyz)),
    )

    ik._draw_curobo(panel)

    assert targets == []
    assert any("measured pose unavailable" in message for message in disabled_messages)


def test_use_current_xyz_copies_a_valid_measured_pose(monkeypatch):
    targets = []
    no_op = lambda *args, **kwargs: None
    monkeypatch.setattr(ik, "begin_disabled_ui", lambda _disabled: None)
    monkeypatch.setattr(ik, "end_disabled_ui", no_op)
    monkeypatch.setattr(ik, "imgui", NS(
        separator=no_op,
        text=no_op,
        text_disabled=no_op,
        text_wrapped=no_op,
        same_line=no_op,
        button=lambda label: label == "Use current XYZ",
    ))
    status = {"available": True, "phase": "idle", "current_xyz": [0.1, -0.2, 0.3]}
    panel = NS(
        service=NS(session=NS(service_value=lambda *_args: status)),
        state=NS(set_target=lambda *xyz: targets.append(xyz)),
    )

    ik._draw_curobo(panel)

    assert targets == [(0.1, -0.2, 0.3)]


def test_settling_status_displays_axis_residual_from_final_goal(monkeypatch):
    messages = []
    no_op = lambda *args, **kwargs: None
    monkeypatch.setattr(ik, "begin_disabled_ui", lambda _disabled: None)
    monkeypatch.setattr(ik, "end_disabled_ui", no_op)
    monkeypatch.setattr(ik, "imgui", NS(
        separator=no_op,
        text=lambda value: messages.append(str(value)),
        text_disabled=lambda value: messages.append(str(value)),
        text_wrapped=lambda value: messages.append(str(value)),
        same_line=no_op,
        button=lambda _label: False,
    ))
    status = {
        "available": True,
        "phase": "settling",
        "tracking": {"error": [0.001, 0.01, 0., 0.],
                     "goal_error": [0.004, 0.08, 0.01, 0.]},
    }
    panel = NS(service=NS(session=NS(service_value=lambda *_args: status)))

    ik._draw_curobo(panel)

    assert any("Goal residual: Linear 4.00 mm" in value for value in messages)
    assert any("limit 8 mm / 0.12 rad" in value for value in messages)


def test_target_status_shows_object_and_planned_claw_centers(monkeypatch):
    messages = []
    no_op = lambda *args, **kwargs: None
    monkeypatch.setattr(ik, "begin_disabled_ui", lambda _disabled: None)
    monkeypatch.setattr(ik, "end_disabled_ui", no_op)
    monkeypatch.setattr(ik, "imgui", NS(
        separator=no_op,
        text=lambda value: messages.append(str(value)),
        text_disabled=lambda value: messages.append(str(value)),
        text_wrapped=lambda value: messages.append(str(value)),
        same_line=no_op,
        button=lambda _label: False,
    ))
    status = {
        "available": True,
        "phase": "ready",
        "target_xyz": [.12, -.01, .5],
        "planned_xyz": [.12, -.01, .5],
    }
    panel = NS(service=NS(session=NS(service_value=lambda *_args: status)))

    ik._draw_curobo(panel)

    assert any("between the open claws" in value for value in messages)
    assert any("Object center XYZ: 0.1200, -0.0100, 0.5000 m" in value for value in messages)
    assert any("Planned claw-center XYZ: 0.1200, -0.0100, 0.5000 m" in value
               for value in messages)
