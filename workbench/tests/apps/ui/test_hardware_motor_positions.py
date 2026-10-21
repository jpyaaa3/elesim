from types import SimpleNamespace as NS

from elesim_ui.panels import hardware


def test_hardware_panel_shows_robot_motor_feedback_keyed_by_dynamixel_id(monkeypatch):
    rendered = []
    monkeypatch.setattr(
        hardware,
        "imgui",
        NS(
            separator=lambda: None,
            text=lambda value: rendered.append(str(value)),
            same_line=lambda *_args: None,
        ),
    )

    hardware._draw_motor_position_rows(
        NS(),
        NS(
            motor_positions_raw={"1": 2048, "2": 6126},
            motor_positions_deg={1: 180.0, 2: 179.8},
        ),
    )

    assert any("linear" in line and "2048" in line and "180.0" in line for line in rendered)
    assert any("roll" in line and "6126" in line and "179.8" in line for line in rendered)


def test_hardware_panel_keeps_accepting_named_motor_feedback(monkeypatch):
    rendered = []
    monkeypatch.setattr(
        hardware,
        "imgui",
        NS(
            separator=lambda: None,
            text=lambda value: rendered.append(str(value)),
            same_line=lambda *_args: None,
        ),
    )

    hardware._draw_motor_position_rows(
        NS(),
        NS(motor_positions_raw={"linear": 3210}, motor_positions_deg={"linear": 281.3}),
    )

    assert any("linear" in line and "3210" in line and "281.3" in line for line in rendered)
