from types import SimpleNamespace as NS

import pytest

from elesim_ui.panels import hardware


def _fake_imgui(*, checked=False, click=False, messages=None):
    messages = messages if messages is not None else []
    return NS(
        separator=lambda: None,
        text=lambda text: messages.append(str(text)),
        text_wrapped=lambda text: messages.append(str(text)),
        checkbox=lambda _label, _value: (bool(checked), bool(checked)),
        button=lambda _label: bool(click),
    )


def test_motor_reboot_button_requires_physical_support_confirmation(monkeypatch):
    calls = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=False, click=True))
    panel = NS(
        _motor_reboot_confirmed=False,
        service=NS(reboot_faulted_motor=lambda **kwargs: calls.append(kwargs)),
    )
    state = NS(
        safety_fault=("Dynamixel ID 5: [RxPacketError] Hardware error occurred."),
        torque_enabled=False,
    )

    hardware._draw_motor_fault_recovery(panel, state)

    assert calls == []
    assert panel._motor_reboot_confirmed is False


def test_motor_reboot_button_requests_robot_recovery_only_after_confirmation(monkeypatch):
    calls = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=True, click=True))
    panel = NS(
        _motor_reboot_confirmed=False,
        service=NS(reboot_faulted_motor=lambda **kwargs: calls.append(kwargs)),
    )
    state = NS(
        safety_fault=("Dynamixel ID 5: [RxPacketError] Hardware error occurred."),
        torque_enabled=False,
    )

    hardware._draw_motor_fault_recovery(panel, state)

    assert calls == [{"arm_supported": True}]
    assert panel._motor_reboot_confirmed is False


def test_motor_reboot_button_is_hidden_when_fault_is_not_dynamixel_shutdown(monkeypatch):
    called = []
    cleared = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=True, click=True))
    panel = NS(
        _motor_reboot_confirmed=True,
        service=NS(
            reboot_faulted_motor=lambda **kwargs: called.append(kwargs),
            clear_safety_fault=lambda: cleared.append(True),
        ),
    )
    state = NS(safety_fault="software current limit exceeded", torque_enabled=False)

    hardware._draw_motor_fault_recovery(panel, state)

    assert called == []
    assert cleared == [True]
    assert panel._motor_reboot_confirmed is False


def test_motor_reboot_requires_torque_off_even_after_support_confirmation(monkeypatch):
    called = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=True, click=True))
    panel = NS(
        _motor_reboot_confirmed=False,
        service=NS(reboot_faulted_motor=lambda **kwargs: called.append(kwargs)),
    )
    state = NS(
        safety_fault=("Dynamixel ID 5: [RxPacketError] Hardware error occurred."),
        torque_enabled=True,
    )

    hardware._draw_motor_fault_recovery(panel, state)

    assert called == []


def test_non_hardware_safety_fault_explains_why_motor_reboot_is_unavailable(monkeypatch):
    messages = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=True, click=True, messages=messages))
    panel = NS(
        _motor_reboot_confirmed=True,
        service=NS(
            reboot_faulted_motor=lambda **kwargs: pytest.fail("must not reboot software fault"),
            clear_safety_fault=lambda: messages.append("clear_requested"),
        ),
    )
    state = NS(
        safety_fault="motor current limit exceeded: ID 1",
        torque_enabled=False,
    )

    hardware._draw_motor_fault_recovery(panel, state)

    assert any("motor current limit exceeded: ID 1" in message for message in messages)
    assert any("does not turn Torque back on" in message for message in messages)
    assert "clear_requested" in messages
    assert panel._motor_reboot_confirmed is False


def test_software_fault_clear_is_not_offered_while_torque_is_enabled(monkeypatch):
    messages = []
    monkeypatch.setattr(hardware, "imgui", _fake_imgui(checked=True, click=True, messages=messages))
    panel = NS(
        _motor_reboot_confirmed=False,
        service=NS(clear_safety_fault=lambda: pytest.fail("must require torque off")),
    )

    hardware._draw_motor_fault_recovery(
        panel,
        NS(safety_fault="motor current limit exceeded: ID 1", torque_enabled=True),
    )

    assert "Turn Torque OFF before clearing the safety fault." in messages
