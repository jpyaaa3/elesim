"""Lease-aware physical robot runtime and independent local safety monitor."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from numbers import Real
from typing import Any, Callable, Optional, Sequence

from elesim_protocol import (
    ControlU,
    Envelope,
    SimQ,
    motor_deg_to_sim_q,
    sim_q_to_motor_deg,
)
from elesim_robot.arm.dynamixel import (
    DXL_CURRENT_UNIT_MA,
    load_hardware,
    tick_to_deg_0_360,
)
from elesim_robot.config import SafetyConfig


@dataclass(frozen=True)
class ArmSnapshot:
    sampled_at: float
    ticks: dict[int, int]
    degrees: dict[int, float]
    currents_ma: dict[int, int]
    q: SimQ


class RobotRuntime:
    """Own physical actuation state; never delegates local safety to the router."""

    def __init__(
        self,
        *,
        mapping: Any,
        hardware_config: Any,
        safety_config: SafetyConfig | None = None,
        device: str = "",
        go2_bridge: Any = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.mapping = mapping
        self.hardware_config = hardware_config
        self.safety = safety_config or SafetyConfig()
        self.device = str(device)
        self.clock = clock
        self.hw: Any = None
        self.go2 = go2_bridge

        self.torque_enabled = False
        self.active_lease = ""
        self.controller_id = ""
        self.last_seq = -1
        self.safety_fault = ""
        self.communication_fault = ""

        self._router_last_seen: Optional[float] = None
        self._go2_command_at: Optional[float] = None
        self._go2_motion_active = False
        self._last_monitor_at: Optional[float] = None
        self._read_failures = 0
        self._read_error = ""
        self._arm_snapshot: Optional[ArmSnapshot] = None

    def open(self) -> None:
        if self.device:
            self.hw, _direction = load_hardware(
                self.device,
                hardware_cfg=self.hardware_config,
            )
            self.hw.open()
        if self.go2 is not None:
            self.go2.start()

    def close(self) -> None:
        self.stop_motion()
        if self.go2 is not None:
            self.go2.stop()
        if self.hw is not None:
            self.hw.close()

    def mark_router_alive(self) -> None:
        self._router_last_seen = self.clock()
        self.communication_fault = ""

    def grant_lease(self, controller_id: str, lease_id: str) -> None:
        self.stop_motion()
        self.controller_id = str(controller_id)
        self.active_lease = str(lease_id)
        self.last_seq = -1
        self.mark_router_alive()

    def revoke_lease(self) -> None:
        self.stop_motion()
        self._clear_lease()

    def _clear_lease(self) -> None:
        self.controller_id = ""
        self.active_lease = ""
        self.last_seq = -1

    def _stop_go2(self) -> None:
        if self.go2 is None:
            return
        self.go2.set_velocity(0.0, 0.0, 0.0)
        self.go2.tick_cmd()
        self._go2_motion_active = False

    def stop_motion(self) -> None:
        self._stop_go2()
        if self.hw is None or not self.torque_enabled:
            return
        try:
            self.hw.safe_hold_arm()
        except Exception as exc:
            self._trip_safety_fault(f"arm safe hold failed: {exc!r}")

    def emergency_stop(self) -> None:
        self._stop_go2()
        if self.hw is not None:
            try:
                self.hw.torque_off_all()
            finally:
                self.torque_enabled = False

    def _trip_safety_fault(self, reason: str) -> None:
        if self.safety_fault:
            return
        self.safety_fault = str(reason)
        self._stop_go2()
        if self.hw is not None:
            try:
                self.hw.torque_off_all()
            except Exception as exc:
                self.safety_fault += f"; torque off failed: {exc!r}"
        self.torque_enabled = False

    def tick(self) -> None:
        now = self.clock()
        self._enforce_router_liveness(now)
        self._enforce_go2_deadman(now)
        self._monitor_hardware(now)
        if self.go2 is not None:
            self.go2.tick_cmd(now)

    def _enforce_router_liveness(self, now: float) -> None:
        if not self.active_lease or self._router_last_seen is None:
            return
        if now - self._router_last_seen <= float(self.safety.router_liveness_s):
            return
        self.stop_motion()
        self._clear_lease()
        self.communication_fault = "router_liveness_timeout"

    def _enforce_go2_deadman(self, now: float) -> None:
        if not self._go2_motion_active or self._go2_command_at is None:
            return
        if now - self._go2_command_at <= float(self.safety.command_deadman_s):
            return
        self._stop_go2()

    def _monitor_hardware(self, now: float, *, force: bool = False) -> None:
        if self.hw is None:
            return
        period = float(self.safety.monitor_period_s)
        if (
            not force
            and self._last_monitor_at is not None
            and now - self._last_monitor_at < period
        ):
            return
        self._last_monitor_at = now
        try:
            snapshot = self._read_arm_snapshot(now)
        except Exception as exc:
            self._read_failures += 1
            self._read_error = repr(exc)
            if self._read_failures >= int(self.safety.read_failure_limit):
                self._trip_safety_fault(
                    f"hardware telemetry unavailable after {self._read_failures} reads: {exc!r}"
                )
            return

        self._arm_snapshot = snapshot
        self._read_failures = 0
        self._read_error = ""
        current_limit = int(getattr(self.hardware_config, "current_limit_ma", 2500))
        over_limit = {
            motor_id: current
            for motor_id, current in snapshot.currents_ma.items()
            if abs(current) > current_limit
        }
        if over_limit:
            self._trip_safety_fault(f"motor current limit exceeded: {over_limit}")

    def _read_arm_snapshot(self, now: float) -> ArmSnapshot:
        ticks = {
            int(motor_id): int(value)
            for motor_id, value in self.hw.get_present_positions().items()
        }
        degrees = {
            motor_id: tick_to_deg_0_360(
                ticks[motor_id],
                int(self.hw.direction.get(motor_id, 1)),
            )
            for motor_id in ticks
        }
        currents_ma = {
            int(motor_id): int(
                round(self.hw.get_present_current(int(motor_id)) * DXL_CURRENT_UNIT_MA)
            )
            for motor_id in self.hw.ids
        }
        config = self.hw.cfg
        arm_ids = (
            int(config.id_linear),
            int(config.id_roll),
            int(config.id_seg1),
            int(config.id_seg2),
        )
        missing = [motor_id for motor_id in arm_ids if motor_id not in degrees]
        if missing:
            raise RuntimeError(f"missing arm position samples: {missing}")
        motor = ControlU(*(degrees[motor_id] for motor_id in arm_ids))
        return ArmSnapshot(
            sampled_at=float(now),
            ticks=ticks,
            degrees=degrees,
            currents_ma=currents_ma,
            q=motor_deg_to_sim_q(motor, self.mapping),
        )

    def apply(self, envelope: Envelope) -> tuple[bool, str]:
        payload = envelope.payload or {}
        command = str(payload.get("command", "")).strip()
        if command == "estop":
            self.emergency_stop()
            return True, "estop"
        if envelope.lease_id != self.active_lease or envelope.source_id != self.controller_id:
            return False, "lease_mismatch"
        if envelope.seq <= self.last_seq:
            return False, "stale_sequence"
        self.last_seq = envelope.seq
        if self.safety_fault and command not in {"torque_off", "clear_fault"}:
            return False, "safety_fault_latched"

        try:
            return self._dispatch(command, payload)
        except Exception as exc:
            self._trip_safety_fault(f"hardware command failed: {exc!r}")
            return False, self.safety_fault

    def _dispatch(self, command: str, payload: dict[str, Any]) -> tuple[bool, str]:
        if command == "target":
            return self._apply_target(payload)
        if command == "torque_on":
            if self.hw is not None:
                self.hw.set_operating_modes()
                self.hw.set_profiles()
                self.hw.torque_on_all()
            self.torque_enabled = True
            return True, "torque_on"
        if command == "torque_off":
            self._stop_go2()
            if self.hw is not None:
                self.hw.torque_off_all()
            self.torque_enabled = False
            return True, "torque_off"
        if command == "clear_fault":
            if self.torque_enabled:
                return False, "torque_must_be_off"
            self.safety_fault = ""
            self._read_failures = 0
            self._monitor_hardware(self.clock(), force=True)
            return (
                (False, "unsafe_to_clear")
                if self.safety_fault
                else (True, "fault_cleared")
            )
        if command == "claw":
            degrees = self._finite_number(payload.get("degrees"), name="claw degrees")
            if not 0.0 <= degrees <= 360.0:
                return False, "claw_out_of_bounds"
            if self.hw is not None:
                self.hw.command_claw_deg(degrees)
            return True, "claw"
        if command == "go2_velocity":
            velocity, reason = self._parse_go2_velocity(
                (payload.get("vx"), payload.get("vy"), payload.get("wz"))
            )
            if velocity is None:
                return False, reason
            self._command_go2_velocity(velocity)
            return True, "go2_velocity"
        return False, "unsupported_command"

    def _apply_target(self, payload: dict[str, Any]) -> tuple[bool, str]:
        if "u" in payload:
            return False, "legacy_u_not_supported"

        q: Optional[SimQ] = None
        if "q" in payload:
            q, reason = self._parse_q(payload["q"])
            if q is None:
                return False, reason

        velocity: Optional[tuple[float, float, float]] = None
        if "go2_vel" in payload:
            velocity, reason = self._parse_go2_velocity(payload["go2_vel"])
            if velocity is None:
                return False, reason

        if "claw_closed" in payload and not isinstance(payload["claw_closed"], bool):
            return False, "bad_claw_state"

        if q is not None and self.hw is not None:
            motor = sim_q_to_motor_deg(q, self.mapping)
            self.hw.command_4dof_deg(
                motor.u_linear,
                motor.u_roll,
                motor.u_s1,
                motor.u_s2,
            )
        if "claw_closed" in payload and self.hw is not None:
            self.hw.command_claw_deg(180.0 if payload["claw_closed"] else 0.0)
        if "go2_sport_pose" in payload and self.go2 is not None:
            pose = str(payload["go2_sport_pose"]).strip()
            if not pose:
                return False, "bad_go2_sport_pose"
            self.go2.call_sport_pose(pose)
        if "go2_obstacles_avoid_enable" in payload and self.go2 is not None:
            enabled = payload["go2_obstacles_avoid_enable"]
            if not isinstance(enabled, bool):
                return False, "bad_go2_obstacles_state"
            self.go2.set_obstacles_avoid(enabled)
        if velocity is not None:
            self._command_go2_velocity(velocity)

        return (True, "target") if q is not None else (True, "target_meta")

    def _parse_q(self, raw: object) -> tuple[Optional[SimQ], str]:
        if not isinstance(raw, (list, tuple)) or len(raw) != 4:
            return None, "bad_q"
        try:
            values = tuple(self._finite_number(value, name="q") for value in raw)
        except ValueError:
            return None, "bad_q"
        bounds = (
            (self.mapping.linear_q_min_m, self.mapping.linear_q_max_m),
            (self.mapping.roll_q_min_rad, self.mapping.roll_q_max_rad),
            (self.mapping.seg1_q_min_rad, self.mapping.seg1_q_max_rad),
            (self.mapping.seg2_q_min_rad, self.mapping.seg2_q_max_rad),
        )
        if any(not min(lower, upper) <= value <= max(lower, upper) for value, (lower, upper) in zip(values, bounds)):
            return None, "q_out_of_bounds"
        return SimQ(*values), ""

    def _parse_go2_velocity(
        self, raw: object
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            return None, "bad_go2_velocity"
        try:
            velocity = tuple(self._finite_number(value, name="go2 velocity") for value in raw)
        except ValueError:
            return None, "bad_go2_velocity"
        limits = (
            float(self.safety.max_go2_vx_m_s),
            float(self.safety.max_go2_vy_m_s),
            float(self.safety.max_go2_wz_rad_s),
        )
        if any(abs(value) > limit for value, limit in zip(velocity, limits)):
            return None, "go2_velocity_out_of_bounds"
        return (float(velocity[0]), float(velocity[1]), float(velocity[2])), ""

    @staticmethod
    def _finite_number(raw: object, *, name: str) -> float:
        if isinstance(raw, bool) or not isinstance(raw, Real):
            raise ValueError(f"{name} must be numeric")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _command_go2_velocity(self, velocity: Sequence[float]) -> None:
        if self.go2 is not None:
            self.go2.set_velocity(float(velocity[0]), float(velocity[1]), float(velocity[2]))
        self._go2_command_at = self.clock()
        self._go2_motion_active = any(abs(float(value)) > 1e-12 for value in velocity)

    def state(self) -> dict[str, object]:
        now = self.clock()
        if self.hw is not None and self._arm_snapshot is None:
            self._monitor_hardware(now, force=True)
        result: dict[str, object] = {
            "device": self.device,
            "torque_enabled": self.torque_enabled,
            "safety_fault": self.safety_fault,
            "communication_fault": self.communication_fault,
            "lease_active": bool(self.active_lease),
            "hardware_read_failures": self._read_failures,
        }
        if self._read_error:
            result["read_error"] = self._read_error
        snapshot = self._arm_snapshot
        if snapshot is not None:
            result.update(
                {
                    "motor_positions_raw": {
                        str(key): value for key, value in snapshot.ticks.items()
                    },
                    "motor_positions_deg": {
                        str(key): value for key, value in snapshot.degrees.items()
                    },
                    "motor_currents_ma": {
                        str(key): value for key, value in snapshot.currents_ma.items()
                    },
                    "q": [
                        snapshot.q.linear_m,
                        snapshot.q.roll_rad,
                        snapshot.q.theta1_rad,
                        snapshot.q.theta2_rad,
                    ],
                    "q_source": "measured",
                    "measurement_monotonic_s": snapshot.sampled_at,
                }
            )
        if self.go2 is not None:
            sample = self.go2.latest_state()
            if sample is not None:
                result["go2"] = {
                    "position": list(sample.pos),
                    "rpy": list(sample.rpy),
                    "linear_velocity_body": list(sample.lin_vel_body),
                    "angular_velocity": list(sample.ang_vel_body),
                    "timestamp": sample.timestamp_s,
                }
        return result


__all__ = ["ArmSnapshot", "RobotRuntime"]
