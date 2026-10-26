from __future__ import annotations

from types import SimpleNamespace

import pytest

from elesim_protocol import ControlU, MockObjectStatePayload, SimulationStatusPayload
from elesim_ui import control_panel
from elesim_ui.panels import go2
from elesim_ui.panels.go2 import (
    _draw_posture,
    _respawn_blocks_teleop,
    _send_go2_velocity,
    _stop_go2,
)
from elesim_ui.sim_view import SimViewState
from elesim_ui.panels import mock_object


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


def test_respawn_blocks_held_teleop_until_all_motion_inputs_are_released() -> None:
    panel = SimpleNamespace(
        _go2_respawn_blocked=True,
        _go2_was_active=True,
        _go2_stop_sent=False,
    )

    assert _respawn_blocks_teleop(panel, active=True, key_stop=False) is True
    assert panel._go2_respawn_blocked is True
    assert panel._go2_was_active is False
    assert panel._go2_stop_sent is True

    assert _respawn_blocks_teleop(panel, active=False, key_stop=False) is False
    assert panel._go2_respawn_blocked is False


def test_latched_go2_fault_stops_teleop_without_polling_held_keys(monkeypatch):
    sent = []
    panel = SimpleNamespace(_host_state=SimpleNamespace(go2_controller_fault='touchdown timeout'),
        _go2_stop_sent=False,_go2_was_active=True,_go2_last_command_at=0.,
        service=SimpleNamespace(send_go2_velocity=lambda **v:sent.append(v)))
    monkeypatch.setattr(go2, '_key_down', lambda *_:pytest.fail('faulted teleop polled held keys'))
    assert not go2._draw_teleop_pad(panel, 280.)
    assert sent == [dict(vx=0.,vy=0.,wz=0.)]


@pytest.mark.parametrize("pose", ("", "stand_up", "balance_stand", "recovery_stand"))
def test_go2_posture_only_shows_stand_and_sit(monkeypatch, pose) -> None:
    labels: list[str] = []
    disabled: list[str] = []
    monkeypatch.setattr(
        go2,
        "imgui",
        SimpleNamespace(
            same_line=lambda: None,
            text_disabled=lambda text: disabled.append(str(text)),
        ),
    )
    monkeypatch.setattr(
        go2,
        "_button",
        lambda _panel, label, _width: labels.append(str(label)) or False,
    )
    panel = SimpleNamespace(
        _host_state=SimpleNamespace(go2_sport_pose=pose),
        service=SimpleNamespace(
            send_go2_velocity=lambda **_kwargs: None,
            send_go2_sport_pose=lambda **_kwargs: None,
        ),
    )

    _draw_posture(panel, 260.0)

    assert labels == ["Stand##go2_stand_up", "Sit##go2_stand_down"]
    assert disabled == []


def test_go2_sit_button_sends_stand_down_even_without_pose_feedback(monkeypatch) -> None:
    sent: list[tuple[str, object]] = []
    monkeypatch.setattr(
        go2,
        "imgui",
        SimpleNamespace(same_line=lambda: None, text_disabled=lambda _text: None),
    )
    monkeypatch.setattr(
        go2,
        "_button",
        lambda _panel, label, _width: str(label) == "Sit##go2_stand_down",
    )
    panel = SimpleNamespace(
        _host_state=SimpleNamespace(go2_sport_pose=""),
        service=SimpleNamespace(
            send_go2_velocity=lambda **kwargs: sent.append(("velocity", kwargs)),
            send_go2_sport_pose=lambda **kwargs: sent.append(("pose", kwargs)),
        ),
    )

    _draw_posture(panel, 260.0)

    assert sent == [
        ("velocity", {"vx": 0.0, "vy": 0.0, "wz": 0.0}),
        ("pose", {"pose": "stand_down"}),
    ]


def test_mock_object_section_remains_visible_without_a_sim_session(monkeypatch) -> None:
    messages: list[str] = []
    fake_imgui = type(
        "Imgui",
        (),
        {
            "input_text": staticmethod(lambda *_args, **_kwargs: (False, "demo_box")),
            "input_float": staticmethod(lambda *_args, **_kwargs: (False, 0.0)),
            "text_disabled": staticmethod(lambda value: messages.append(str(value))),
            "button": staticmethod(lambda *_args, **_kwargs: False),
            "same_line": staticmethod(lambda: None),
            "text_wrapped": staticmethod(lambda value: messages.append(str(value))),
        },
    )
    monkeypatch.setattr(mock_object, "imgui", fake_imgui)
    monkeypatch.setattr(mock_object, "panel_header", lambda *_args, **_kwargs: (True, True))
    panel = type(
        "Panel",
        (),
        {
            "_sim_view": None,
            "_mock_object_asset": "demo_box",
            "_mock_object_position": [0.5, 0.0, 0.4],
            "_mock_object_euler_deg": [0.0, 0.0, 0.0],
            "_mock_hug_solution": {},
            "_mock_object_message": "",
        },
    )()
    mock_object.draw_mock_object_panel(panel)
    assert any("unavailable" in value for value in messages)


def test_mock_object_panel_keeps_spawn_authority_separate_from_hug_motion(
    monkeypatch,
) -> None:
    pressed: set[str] = {"Spawn##mock-object-spawn", "Compute hug##mock-hug-compute"}
    fake_imgui = type(
        "Imgui",
        (),
        {
            "input_text": staticmethod(lambda *_args, **_kwargs: (False, "demo_box.obj")),
            "input_float": staticmethod(lambda *_args, **_kwargs: (False, 0.0)),
            "text_disabled": staticmethod(lambda *_args, **_kwargs: None),
            "button": staticmethod(lambda label: label in pressed),
            "same_line": staticmethod(lambda: None),
            "text_wrapped": staticmethod(lambda *_args, **_kwargs: None),
        },
    )
    monkeypatch.setattr(mock_object, "imgui", fake_imgui)
    monkeypatch.setattr(mock_object, "panel_header", lambda *_args, **_kwargs: (True, True))
    commands: list[tuple[str, object]] = []
    motion_calls: list[tuple[str, object]] = []
    status = SimulationStatusPayload(
        0,
        False,
        1.0,
        True,
        0.0,
        mock_object=MockObjectStatePayload(
            available_assets=("demo_box.obj",),
            state="spawned",
            asset_id="demo_box",
            revision=1,
            sha256="a" * 64,
            silhouette_xz=((-0.1, -0.1), (0.1, -0.1), (0.0, 0.1)),
        ),
    )

    class Session:
        snapshot = SimpleNamespace(status=status)

        def send_command(self, name, arguments=None):
            commands.append((name, arguments))

    class Service:
        def compute_mock_hug(self, **kwargs):
            motion_calls.append(("compute", None))
            kwargs["on_result"]({"solution_id": "solution-a"})

        def execute_mock_hug(self, solution_id, **kwargs):
            motion_calls.append(("execute", solution_id))
            kwargs["on_result"]({"state": "executing"})

    panel = SimpleNamespace(
        _sim_view=SimpleNamespace(session=Session()),
        _mock_object_asset="demo_box.obj",
        _mock_object_position=[0.5, 0.0, 0.4],
        _mock_object_euler_deg=[0.0, 0.0, 0.0],
        _mock_hug_solution={},
        _mock_object_message="",
        service=Service(),
    )
    mock_object.draw_mock_object_panel(panel)
    pressed.clear()
    pressed.add("Execute##mock-hug-execute")
    mock_object.draw_mock_object_panel(panel)

    assert commands == [
        (
            "spawn_mock_object",
            {
                "asset_id": "demo_box.obj",
                "position": [0.5, 0.0, 0.4],
                "euler_deg": [0.0, 0.0, 0.0],
            },
        )
    ]
    assert motion_calls == [("compute", None), ("execute", "solution-a")]
