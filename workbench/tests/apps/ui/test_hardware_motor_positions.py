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


def test_hardware_current_table_uses_motor_ids_and_keeps_gripper_load_separate(monkeypatch):
    from elesim_ui.panels import live_visual_status as view
    tables, lines = {}, []
    monkeypatch.setattr(view, "_draw_columns_table", lambda panel, key, labels, values, **kwargs: tables.__setitem__(key, values))
    monkeypatch.setattr(view, "_draw_go2_torque_table", lambda *args: None)
    monkeypatch.setattr(view, "_current_color", lambda *args: None)
    monkeypatch.setattr(view, "_line", lambda *args, **kwargs: lines.append(args))
    host = NS(connected=True, rx_age_s=0., motor_currents_ma={"1": -97, 2: 0, "3": 123, "4": -42},
        claw_load_permille=225)
    panel = NS(_host_state=host, state=NS(perception_running=False))
    view._draw_hardware_brief(panel)
    values = tables["hardware_current_table"]
    assert values[:4] == tuple(view._fmt_ma(v) for v in (-97, 0, 123, -42))
    assert values[4] == "-"
    assert any(line[0] == "Gripper estimated load" for line in lines)
    host.motor_currents_ma["5"] = 17
    view._draw_hardware_brief(panel)
    assert tables["hardware_current_table"][4] == view._fmt_ma(17)
