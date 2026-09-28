"""Native arm mapping must retain the existing Robot wire-to-motor meaning."""

from __future__ import annotations

import ctypes
import math
import time
import subprocess
import sys
from pathlib import Path

import pytest

from elesim_protocol import ImuModelDefinition, SimMappingConfig, SimQ, sim_q_to_motor_deg
from elesim_robot.arm.dynamixel import NativeArm, _load_library, _native_config, _native_program
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


def test_native_identity_model_and_imu_input_are_validated(native_library: Path) -> None:
    arm = NativeArm(
        "/dev/elesim-no-such-dynamixel-bus",
        hardware=HardwareConfig(), mapping=SimMappingConfig(),
        safety=SafetyConfig(), library_path=native_library,
    )
    try:
        assert arm.snapshot().model_id == "identity"
        identity = ImuModelDefinition.from_payload({
            "schema_version": 1, "id": "identity", "version": 1,
            "program": {"nodes": [
                {"op": "q", "index": index} for index in range(4)
            ], "outputs": [0, 1, 2, 3]},
        })
        arm.select_imu_model(identity)
        arm.submit_imu((0.1, 0.2, 0.3), time.monotonic())
        assert arm.snapshot().imu_valid is True
        with pytest.raises(RuntimeError, match="stale IMU sample"):
            arm.submit_imu((0.1, 0.2, 0.3), 1.0)
        with pytest.raises(RuntimeError, match="invalid IMU sample"):
            arm.submit_imu((math.nan, 0.0, 0.0), time.monotonic())
    finally:
        arm.close()


def test_native_formula_is_loaded_from_model_data(native_library: Path) -> None:
    model = ImuModelDefinition.from_payload({
        "schema_version": 1, "id": "imu_gain", "version": 2,
        "program": {"nodes": [
            {"op": "q", "index": 0},
            {"op": "q", "index": 1},
            {"op": "q", "index": 2},
            {"op": "q", "index": 3},
            {"op": "imu", "index": 0},
            {"op": "const", "value": 0.25},
            {"op": "mul", "a": 4, "b": 5},
            {"op": "add", "a": 1, "b": 6},
        ], "outputs": [0, 7, 2, 3]},
    })
    library = _load_library(native_library)
    program = _native_program(model)
    q = (ctypes.c_double * 4)(-0.1, 0.2, 0.3, -0.4)
    imu = (ctypes.c_double * 3)(0.4, 0.0, 0.0)
    output = (ctypes.c_double * 4)()
    assert library.elesim_arm_eval_model(ctypes.byref(program), q, imu, 0, output) != 0
    assert library.elesim_arm_eval_model(ctypes.byref(program), q, imu, 1, output) == 0
    assert list(output) == pytest.approx([-0.1, 0.3, 0.3, -0.4])
