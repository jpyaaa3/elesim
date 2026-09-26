"""Native arm mapping must retain the existing Robot wire-to-motor meaning."""

from __future__ import annotations

import ctypes
import math
import subprocess
import sys
from pathlib import Path

import pytest

from elesim_protocol import SimMappingConfig, SimQ, sim_q_to_motor_deg
from elesim_robot.arm.dynamixel import NativeArm, _load_library, _native_config
from elesim_robot.config import HardwareConfig, SafetyConfig


ROOT = Path(__file__).resolve().parents[4]
BUILDER = ROOT / "payload/runtime/native/robot/native_arm/build.py"


@pytest.fixture(scope="module")
def native_library(tmp_path_factory: pytest.TempPathFactory) -> Path:
    output = tmp_path_factory.mktemp("robot-native-arm") / "libelesim_arm.so"
    subprocess.run([sys.executable, str(BUILDER), str(output)], check=True, capture_output=True)
    return output


def test_native_placeholder_preserves_canonical_q_mapping(native_library: Path) -> None:
    library = _load_library(native_library)
    mapping = SimMappingConfig()
    config = _native_config(HardwareConfig(), mapping, SafetyConfig())
    for q in (
        SimQ(mapping.linear_q_min_m, mapping.roll_q_min_rad, mapping.seg1_q_min_rad, mapping.seg2_q_min_rad),
        SimQ(-0.1, 0.0, 0.1, -0.1),
        SimQ(mapping.linear_q_max_m, mapping.roll_q_max_rad, mapping.seg1_q_max_rad, mapping.seg2_q_max_rad),
    ):
        raw = (ctypes.c_double * 4)(q.linear_m, q.roll_rad, q.theta1_rad, q.theta2_rad)
        output = (ctypes.c_double * 4)()
        assert library.elesim_arm_map_q(ctypes.byref(config), raw, output) == 0
        expected = sim_q_to_motor_deg(q, mapping)
        assert list(output) == pytest.approx(
            [expected.u_linear, expected.u_roll, expected.u_s1, expected.u_s2]
        )


def test_native_mapping_rejects_nonfinite_and_out_of_range_q(native_library: Path) -> None:
    library = _load_library(native_library)
    config = _native_config(HardwareConfig(), SimMappingConfig(), SafetyConfig())
    output = (ctypes.c_double * 4)()
    for q in ((math.nan, 0.0, 0.0, 0.0), (-1.0, 0.0, 0.0, 0.0)):
        raw = (ctypes.c_double * 4)(*q)
        assert library.elesim_arm_map_q(ctypes.byref(config), raw, output) != 0


def test_missing_native_controller_fails_closed(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match=r"C\+\+ arm controller is missing"):
        NativeArm(
            "/dev/ttyUSB0",
            hardware=HardwareConfig(),
            mapping=SimMappingConfig(),
            safety=SafetyConfig(),
            library_path=tmp_path / "missing.so",
        )


def test_native_controller_rejects_unavailable_bus(native_library: Path) -> None:
    arm = NativeArm(
        "/dev/elesim-no-such-dynamixel-bus",
        hardware=HardwareConfig(),
        mapping=SimMappingConfig(),
        safety=SafetyConfig(),
        library_path=native_library,
    )
    try:
        with pytest.raises(RuntimeError, match="failed to open Dynamixel bus"):
            arm.open()
    finally:
        arm.close()
