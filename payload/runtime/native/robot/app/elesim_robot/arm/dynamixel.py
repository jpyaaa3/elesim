"""Thin Robot process binding for the Jetson-local C++ arm controller."""

from __future__ import annotations

import ctypes
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from elesim_protocol import SimMappingConfig

if TYPE_CHECKING:
    from elesim_robot.config import HardwareConfig, SafetyConfig


DXL_CURRENT_UNIT_MA = 2.69
TICK_MAX = 4095
MOTOR_IDS = (1, 2, 3, 4, 5)


def deg_to_tick_0_360(degrees: float) -> int:
    return max(0, min(TICK_MAX, round(max(0.0, min(360.0, degrees)) * TICK_MAX / 360.0)))


def tick_to_deg_0_360(tick: int, direction: int = 1) -> float:
    limited = max(0, min(TICK_MAX, int(tick)))
    return (TICK_MAX - limited if direction < 0 else limited) * 360.0 / TICK_MAX


class _ArmConfig(ctypes.Structure):
    _fields_ = [
        ("baudrate", ctypes.c_int32),
        ("motor_direction", ctypes.c_int32 * 4),
        ("profile_velocity", ctypes.c_int32 * 5),
        ("profile_acceleration", ctypes.c_int32 * 5),
        ("current_limit_ma", ctypes.c_int32),
        ("read_failure_limit", ctypes.c_int32),
        ("monitor_period_s", ctypes.c_double),
        ("linear_motor_limit_deg", ctypes.c_double),
        ("q_min", ctypes.c_double * 4),
        ("q_max", ctypes.c_double * 4),
        ("motor_min_deg", ctypes.c_double * 4),
        ("motor_max_deg", ctypes.c_double * 4),
    ]


class _ArmSnapshot(ctypes.Structure):
    _fields_ = [
        ("sampled_monotonic_s", ctypes.c_double),
        ("ticks", ctypes.c_int32 * 5),
        ("currents_ma", ctypes.c_int32 * 5),
        ("torque_enabled", ctypes.c_int32),
        ("read_failures", ctypes.c_int32),
        ("valid", ctypes.c_int32),
    ]


@dataclass(frozen=True)
class NativeArmState:
    sampled_at: float
    ticks: dict[int, int]
    currents_ma: dict[int, int]
    torque_enabled: bool
    read_failures: int
    fault: str
    valid: bool


def _native_config(
    hardware: HardwareConfig,
    mapping: SimMappingConfig,
    safety: SafetyConfig,
) -> _ArmConfig:
    return _ArmConfig(
        int(hardware.baudrate),
        (ctypes.c_int32 * 4)(*hardware.motor_direction),
        (ctypes.c_int32 * 5)(
            hardware.profile_vel_linear,
            hardware.profile_vel_roll,
            hardware.profile_vel_seg1,
            hardware.profile_vel_seg2,
            hardware.profile_vel_claw,
        ),
        (ctypes.c_int32 * 5)(
            hardware.profile_acc_linear,
            hardware.profile_acc_roll,
            hardware.profile_acc_seg1,
            hardware.profile_acc_seg2,
            hardware.profile_acc_claw,
        ),
        int(hardware.current_limit_ma),
        int(safety.read_failure_limit),
        float(safety.monitor_period_s),
        float(hardware.linear_u_limit_deg),
        (ctypes.c_double * 4)(
            mapping.linear_q_min_m,
            mapping.roll_q_min_rad,
            mapping.seg1_q_min_rad,
            mapping.seg2_q_min_rad,
        ),
        (ctypes.c_double * 4)(
            mapping.linear_q_max_m,
            mapping.roll_q_max_rad,
            mapping.seg1_q_max_rad,
            mapping.seg2_q_max_rad,
        ),
        (ctypes.c_double * 4)(
            mapping.linear_u_min,
            mapping.roll_u_min,
            mapping.seg_u_min,
            mapping.seg_u_min,
        ),
        (ctypes.c_double * 4)(
            mapping.linear_u_max,
            mapping.roll_u_max,
            mapping.seg_u_max,
            mapping.seg_u_max,
        ),
    )


def _load_library(path: Path | None = None) -> ctypes.CDLL:
    location = path or Path(sys.prefix).parent / "native/libelesim_arm.so"
    if not location.is_file():
        raise RuntimeError(f"Robot C++ arm controller is missing: {location}")
    library = ctypes.CDLL(str(location))
    pointer = ctypes.c_void_p
    error = ctypes.c_void_p
    size = ctypes.c_size_t
    library.elesim_arm_create.argtypes = [ctypes.c_char_p, ctypes.POINTER(_ArmConfig), error, size]
    library.elesim_arm_create.restype = pointer
    for name in (
        "elesim_arm_open",
        "elesim_arm_torque_on",
        "elesim_arm_torque_off",
        "elesim_arm_safe_hold",
        "elesim_arm_clear_fault",
        "elesim_arm_close",
    ):
        function = getattr(library, name)
        function.argtypes = [pointer, error, size]
        function.restype = ctypes.c_int
    library.elesim_arm_command_q.argtypes = [pointer, ctypes.POINTER(ctypes.c_double), error, size]
    library.elesim_arm_command_q.restype = ctypes.c_int
    library.elesim_arm_command_claw.argtypes = [pointer, ctypes.c_double, error, size]
    library.elesim_arm_command_claw.restype = ctypes.c_int
    library.elesim_arm_snapshot.argtypes = [pointer, ctypes.POINTER(_ArmSnapshot), error, size]
    library.elesim_arm_snapshot.restype = ctypes.c_int
    library.elesim_arm_destroy.argtypes = [pointer]
    library.elesim_arm_destroy.restype = None
    library.elesim_arm_map_q.argtypes = [
        ctypes.POINTER(_ArmConfig),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    library.elesim_arm_map_q.restype = ctypes.c_int
    return library


class NativeArm:
    """No Python motor packets: only target/status calls cross this boundary."""

    native_safety = True

    def __init__(
        self,
        device: str,
        *,
        hardware: HardwareConfig,
        mapping: SimMappingConfig,
        safety: SafetyConfig,
        library_path: Path | None = None,
    ) -> None:
        self._library = _load_library(library_path)
        self._config = _native_config(hardware, mapping, safety)
        error = ctypes.create_string_buffer(512)
        self._handle = self._library.elesim_arm_create(
            device.encode("utf-8"), ctypes.byref(self._config), error, len(error)
        )
        if not self._handle:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        self.ids = list(MOTOR_IDS)
        self.direction = {
            motor_id: int(hardware.motor_direction[index])
            for index, motor_id in enumerate(MOTOR_IDS[:4])
        }
        self.direction[5] = 1
        self.cfg = type("ArmIds", (), dict(id_linear=1, id_roll=2, id_seg1=3, id_seg2=4, id_claw=5))()

    def _call(self, name: str, *args: Any) -> None:
        if not self._handle:
            raise RuntimeError("Robot C++ arm controller is closed")
        error = ctypes.create_string_buffer(512)
        result = getattr(self._library, name)(self._handle, *args, error, len(error))
        if result != 0:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))

    def open(self) -> None:
        self._call("elesim_arm_open")

    def close(self) -> None:
        if not self._handle:
            return
        try:
            self._call("elesim_arm_close")
        finally:
            self._library.elesim_arm_destroy(self._handle)
            self._handle = None

    def command_q(self, q: tuple[float, float, float, float]) -> None:
        values = (ctypes.c_double * 4)(*q)
        self._call("elesim_arm_command_q", values)

    def command_claw_deg(self, degrees: float) -> None:
        self._call("elesim_arm_command_claw", float(degrees))

    def torque_on_all(self) -> None:
        self._call("elesim_arm_torque_on")

    def torque_off_all(self) -> None:
        self._call("elesim_arm_torque_off")

    def safe_hold_arm(self) -> None:
        self._call("elesim_arm_safe_hold")

    def clear_fault(self) -> None:
        self._call("elesim_arm_clear_fault")

    def snapshot(self) -> NativeArmState:
        if not self._handle:
            raise RuntimeError("Robot C++ arm controller is closed")
        result = _ArmSnapshot()
        fault = ctypes.create_string_buffer(512)
        if self._library.elesim_arm_snapshot(self._handle, ctypes.byref(result), fault, len(fault)) != 0:
            raise RuntimeError(fault.value.decode("utf-8", errors="replace"))
        return NativeArmState(
            sampled_at=float(result.sampled_monotonic_s),
            ticks={key: int(result.ticks[index]) for index, key in enumerate(MOTOR_IDS)},
            currents_ma={key: int(result.currents_ma[index]) for index, key in enumerate(MOTOR_IDS)},
            torque_enabled=bool(result.torque_enabled),
            read_failures=int(result.read_failures),
            fault=fault.value.decode("utf-8", errors="replace"),
            valid=bool(result.valid),
        )


def load_hardware(
    device: str,
    *,
    hardware_cfg: HardwareConfig,
    mapping_cfg: SimMappingConfig,
    safety_cfg: SafetyConfig,
) -> tuple[NativeArm, dict[int, int]]:
    hardware = NativeArm(
        device,
        hardware=hardware_cfg,
        mapping=mapping_cfg,
        safety=safety_cfg,
    )
    return hardware, dict(hardware.direction)


__all__ = [
    "DXL_CURRENT_UNIT_MA",
    "NativeArm",
    "NativeArmState",
    "deg_to_tick_0_360",
    "tick_to_deg_0_360",
    "load_hardware",
]
