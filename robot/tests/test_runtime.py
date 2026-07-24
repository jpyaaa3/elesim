from __future__ import annotations

import math
from dataclasses import replace

import pytest

from elesim_protocol import (
    ControlU,
    Envelope,
    SimMappingConfig,
    make_envelope,
    motor_deg_to_sim_q,
)
from elesim_robot.arm.dynamixel import DXL_CURRENT_UNIT_MA, deg_to_tick_0_360
from elesim_robot.config import HardwareConfig, SafetyConfig
from elesim_robot.runtime import RobotRuntime


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += float(seconds)


class FakeArm:
    class Config:
        id_linear = 1
        id_roll = 2
        id_seg1 = 3
        id_seg2 = 4
        id_claw = 5

    def __init__(self) -> None:
        self.cfg = self.Config()
        self.ids = [1, 2, 3, 4, 5]
        self.direction = {motor_id: 1 for motor_id in self.ids}
        self.target: tuple[float, ...] | None = None
        self.safe_hold_count = 0
        self.torque_off_count = 0
        self.torque_on_count = 0
        self.position_mode_count = 0
        self.positions = {motor_id: deg_to_tick_0_360(180.0) for motor_id in self.ids}
        self.currents_raw = {motor_id: 0 for motor_id in self.ids}
        self.read_error: Exception | None = None

    def command_4dof_deg(self, *target: float) -> None:
        self.target = tuple(target)

    def command_claw_deg(self, _degrees: float) -> None:
        pass

    def safe_hold_arm(self) -> None:
        self.safe_hold_count += 1

    def torque_off_all(self) -> None:
        self.torque_off_count += 1

    def torque_on_all(self) -> None:
        self.torque_on_count += 1

    def set_operating_modes(self) -> None:
        self.position_mode_count += 1

    def set_profiles(self) -> None:
        pass

    def get_present_positions(self) -> dict[int, int]:
        if self.read_error is not None:
            raise self.read_error
        return dict(self.positions)

    def get_present_current(self, motor_id: int) -> int:
        if self.read_error is not None:
            raise self.read_error
        return int(self.currents_raw[motor_id])


class FakeGo2:
    def __init__(self) -> None:
        self.velocities: list[tuple[float, float, float]] = []
        self.tick_times: list[float | None] = []

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self.velocities.append((float(vx), float(vy), float(wz)))

    def tick_cmd(self, now: float | None = None) -> None:
        self.tick_times.append(now)

    def latest_state(self):
        return None

    def call_sport_pose(self, _name: str) -> None:
        pass

    def set_obstacles_avoid(self, _enabled: bool) -> None:
        pass


def command(payload: dict[str, object], *, seq: int = 2, lease: str = "lease-a") -> Envelope:
    return make_envelope(
        "motion_command",
        "controller-a",
        target_id="robot-a",
        payload=payload,
        seq=seq,
        lease_id=lease,
    )


def runtime(
    *,
    clock: FakeClock | None = None,
    safety: SafetyConfig | None = None,
    go2: FakeGo2 | None = None,
) -> tuple[RobotRuntime, FakeArm]:
    arm = FakeArm()
    value = RobotRuntime(
        mapping=SimMappingConfig(),
        hardware_config=HardwareConfig(),
        safety_config=safety or SafetyConfig(),
        go2_bridge=go2,
        clock=clock or FakeClock(),
    )
    value.hw = arm
    return value, arm


def test_runtime_accepts_only_canonical_q_target() -> None:
    value, arm = runtime()
    value.grant_lease("controller-a", "lease-a")

    ok, reason = value.apply(command({"command": "target", "q": [-0.1, 0.0, 0.1, -0.1]}))
    assert (ok, reason) == (True, "target")
    assert arm.target is not None

    ok, reason = value.apply(
        command(
            {"command": "target", "u": {"linear": 10, "roll": 20, "s1": 30, "s2": 40}},
            seq=3,
        )
    )
    assert (ok, reason) == (False, "legacy_u_not_supported")


@pytest.mark.parametrize(
    "q, reason",
    (
        ([-0.1, 0.0, 0.0], "bad_q"),
        ([-1.0, 0.0, 0.0, 0.0], "q_out_of_bounds"),
        ([-0.1, 10.0, 0.0, 0.0], "q_out_of_bounds"),
    ),
)
def test_runtime_rejects_nonfinite_or_out_of_range_q_without_hardware_write(
    q: list[float], reason: str
) -> None:
    value, arm = runtime()
    value.grant_lease("controller-a", "lease-a")

    assert value.apply(command({"command": "target", "q": q})) == (False, reason)
    assert arm.target is None


def test_runtime_rejects_wrong_lease_without_touching_hardware() -> None:
    value, arm = runtime()
    value.grant_lease("controller-a", "lease-a")
    ok, reason = value.apply(command({"command": "target", "q": [0, 0, 0, 0]}, lease="wrong"))
    assert (ok, reason) == (False, "lease_mismatch")
    assert arm.target is None


def test_lease_revoke_holds_position_mode_arm_instead_of_writing_velocity_register() -> None:
    value, arm = runtime()
    value.grant_lease("controller-a", "lease-a")
    value.torque_enabled = True

    value.revoke_lease()

    assert arm.safe_hold_count == 1
    assert value.active_lease == ""


def test_measured_motor_positions_are_reported_as_canonical_q() -> None:
    value, arm = runtime()
    motor = ControlU(125.0, 90.0, 270.0, 180.0)
    arm.positions.update(
        {
            1: deg_to_tick_0_360(motor.u_linear),
            2: deg_to_tick_0_360(motor.u_roll),
            3: deg_to_tick_0_360(motor.u_s1),
            4: deg_to_tick_0_360(motor.u_s2),
        }
    )

    value.tick()
    state = value.state()

    expected = motor_deg_to_sim_q(motor, value.mapping)
    assert state["q"] == pytest.approx(
        [expected.linear_m, expected.roll_rad, expected.theta1_rad, expected.theta2_rad],
        abs=5e-4,
    )
    assert state["q_source"] == "measured"


def test_overcurrent_trips_without_controller_or_telemetry_request() -> None:
    safety = replace(SafetyConfig(), monitor_period_s=0.0)
    value, arm = runtime(safety=safety)
    value.torque_enabled = True
    arm.currents_raw[3] = int(math.ceil(3000.0 / DXL_CURRENT_UNIT_MA))

    value.tick()

    assert "current limit exceeded" in value.safety_fault
    assert arm.torque_off_count == 1
    assert value.torque_enabled is False


def test_repeated_hardware_read_failure_is_fail_safe_without_lease() -> None:
    safety = replace(SafetyConfig(), monitor_period_s=0.0, read_failure_limit=2)
    value, arm = runtime(safety=safety)
    value.torque_enabled = True
    arm.read_error = RuntimeError("serial disconnected")

    value.tick()
    assert value.safety_fault == ""
    value.tick()

    assert "hardware telemetry unavailable" in value.safety_fault
    assert arm.torque_off_count == 1


def test_safety_fault_is_latched_and_blocks_torque_on() -> None:
    value, arm = runtime(safety=replace(SafetyConfig(), monitor_period_s=0.0))
    value.grant_lease("controller-a", "lease-a")
    value.torque_enabled = True
    arm.currents_raw[1] = int(math.ceil(3000.0 / DXL_CURRENT_UNIT_MA))
    value.tick()

    assert value.apply(command({"command": "torque_on"})) == (False, "safety_fault_latched")
    assert arm.torque_on_count == 0


def test_go2_velocity_deadman_is_not_extended_by_unrelated_arm_commands() -> None:
    clock = FakeClock()
    go2 = FakeGo2()
    safety = replace(SafetyConfig(), command_deadman_s=0.5, monitor_period_s=10.0)
    value, _arm = runtime(clock=clock, safety=safety, go2=go2)
    value.grant_lease("controller-a", "lease-a")
    go2.velocities.clear()
    assert value.apply(command({"command": "go2_velocity", "vx": 0.2, "vy": 0.0, "wz": 0.0}))[0]
    clock.advance(0.4)
    assert value.apply(command({"command": "target", "q": [-0.1, 0.0, 0.0, 0.0]}, seq=3))[0]
    clock.advance(0.11)

    value.tick()

    assert go2.velocities[-1] == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "velocity",
    ((100.0, 0.0, 0.0),),
)
def test_invalid_go2_velocity_is_rejected_atomically(velocity: tuple[float, float, float]) -> None:
    go2 = FakeGo2()
    value, _arm = runtime(go2=go2)
    value.grant_lease("controller-a", "lease-a")
    go2.velocities.clear()

    ok, _reason = value.apply(
        command(
            {"command": "go2_velocity", "vx": velocity[0], "vy": velocity[1], "wz": velocity[2]}
        )
    )

    assert ok is False
    assert go2.velocities == []
