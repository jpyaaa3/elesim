from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from elesim_robot.arm.dynamixel import Dynamixel3dofDriver


def driver(mode: str) -> Dynamixel3dofDriver:
    value = object.__new__(Dynamixel3dofDriver)
    value.arm_mode = mode
    value.stop_arm_velocity = MagicMock()
    value.stop_lji_velocity = MagicMock()
    value.set_position_mode_for_arm = MagicMock(side_effect=lambda: setattr(value, "arm_mode", "position"))
    value.hold_current_arm_position = MagicMock()
    return value


def test_safe_hold_in_position_mode_only_updates_position_goal() -> None:
    value = driver("position")
    value.safe_hold_arm()
    value.stop_arm_velocity.assert_not_called()
    value.set_position_mode_for_arm.assert_not_called()
    value.hold_current_arm_position.assert_called_once_with()


@pytest.mark.parametrize("mode, stop_method", (("velocity", "stop_arm_velocity"), ("hybrid", "stop_lji_velocity")))
def test_safe_hold_zeroes_velocity_before_switching_to_position(mode: str, stop_method: str) -> None:
    value = driver(mode)
    value.safe_hold_arm()
    getattr(value, stop_method).assert_called_once_with()
    value.set_position_mode_for_arm.assert_called_once_with()
    value.hold_current_arm_position.assert_called_once_with()
    assert value.arm_mode == "position"
