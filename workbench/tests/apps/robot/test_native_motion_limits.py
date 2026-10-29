import ctypes
import math
import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

from elesim_protocol import SimMappingConfig
from elesim_robot.arm.dynamixel import NativeArm, _native_config
from elesim_robot.config import HardwareConfig, SafetyConfig

ROOT = Path(__file__).resolve().parents[4]
NATIVE = ROOT / "payload/runtime/native/robot/native_arm"


@pytest.fixture(scope="module")
def controller_library(tmp_path_factory):
    root = tmp_path_factory.mktemp("native-sdk-double")
    include = root / "dynamixel_sdk"
    include.mkdir()
    shutil.copyfile(Path(__file__).with_name("native_sdk_stub.h"), include / "dynamixel_sdk.h")
    wrapper = root / "probe.cpp"
    wrapper.write_text('''
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motion_limits.h"
extern "C" unsigned fake_read(int id, int address) { return dynamixel::registers[id][address]; }
extern "C" int fake_bad_enable() { return dynamixel::enabled_before_hold; }
extern "C" void fake_set_current_raw(int id, unsigned value) {
  dynamixel::registers[id][126] = value;
}
extern "C" void fake_set_load_raw(int id, unsigned value) {
  dynamixel::registers[id][126] = value;
}
extern "C" void fake_set_position(int id, unsigned value) {
  dynamixel::registers[id][132] = value;
}
extern "C" void fake_set_torque(int id, unsigned value) {
  dynamixel::registers[id][64] = value;
}
extern "C" void fake_set_hardware_error(int id, int value) {
  dynamixel::hardware_errors[id] = value ? 1 : 0;
}
extern "C" int fake_reboot_count(int id) { return dynamixel::reboot_count[id]; }
extern "C" void fake_reset() {
  dynamixel::enabled_before_hold = 0;
  for (int id=1; id<=5; ++id) {
    dynamixel::hardware_errors[id] = 0;
    dynamixel::reboot_count[id] = 0;
    for (int address=0; address<256; ++address) dynamixel::registers[id][address] = 0;
    dynamixel::registers[id][132] = id == 5 ? 3000 : 2048;
  }
}
extern "C" double probe_stalled_step(double elapsed) {
  double rates[4] = {10,10,10,10};
  return elesim_arm::limit_motor_step({0,0,0,0},{100,100,100,100},rates,elapsed)[0];
}
''')
    library = root / "libelesim_arm.so"
    subprocess.run(["c++", "-std=c++17", "-shared", "-fPIC", "-pthread", "-O1",
                    "-I", str(root), "-I", str(NATIVE), str(NATIVE / "control.cpp"),
                    str(NATIVE / "correction_model.cpp"), str(wrapper), "-o", str(library)],
                   check=True, capture_output=True)
    return library


def make_arm(path, *, safety=None, hardware=None):
    arm = NativeArm("test-only-no-hardware", hardware=hardware or HardwareConfig(), mapping=SimMappingConfig(),
                    safety=safety or SafetyConfig(), library_path=path)
    lib = arm._library
    lib.fake_reset()
    lib.fake_read.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.fake_read.restype = ctypes.c_uint
    arm.open()
    return arm, lib


def test_torque_on_preloads_measured_hold_before_enabling_any_motor(controller_library):
    arm, lib = make_arm(controller_library)
    try:
        arm.torque_on_all()
        assert lib.fake_bad_enable() == 0
        for motor in range(1, 6):
            assert lib.fake_read(motor, 116) == lib.fake_read(motor, 132)
    finally:
        arm.close()


@pytest.mark.parametrize("roll", [-0.3, 0.3])
@pytest.mark.parametrize("direction", [-1, 1])
def test_roll_motor_polarity_matches_command_and_measured_feedback(controller_library, roll, direction):
    """Configured Roll polarity must match native command and readback."""
    from types import SimpleNamespace
    from elesim_robot.runtime import RobotRuntime

    hardware = HardwareConfig(motor_direction=(1, direction, 1, -1))
    arm, lib = make_arm(controller_library, hardware=hardware)
    try:
        assert arm.direction[2] == direction
        arm.torque_on_all()
        start = lib.fake_read(2, 116)
        arm.command_q((-0.1, roll, 0.0, 0.0))
        deadline = time.monotonic() + 0.5
        goal = start
        while goal == start and time.monotonic() < deadline:
            time.sleep(0.01)
            goal = lib.fake_read(2, 116)
        assert (goal - start) * roll * direction > 0

        # Feed back the SDK goal as a measured encoder sample through the
        # production telemetry conversion, without moving a physical motor.
        raw_ticks = {motor: lib.fake_read(motor, 132) for motor in range(1, 6)}
        raw_ticks[2] = goal
        sample = SimpleNamespace(
            ticks=raw_ticks, sampled_at=0.0, currents_ma={}, claw_load_permille=0,
        )
        runtime = SimpleNamespace(hw=arm, mapping=SimMappingConfig())
        snapshot = RobotRuntime._arm_snapshot_from_native(runtime, sample)
        assert snapshot.q.roll_rad * roll > 0
        canonical_ticks = goal if direction > 0 else 4095 - goal
        expected = (canonical_ticks / 4095.0 - 0.5) * math.pi
        assert snapshot.q.roll_rad == pytest.approx(expected)
    finally:
        arm.close()


def test_target_cannot_bypass_slew_limit_and_torque_off_discards_it(controller_library):
    arm, lib = make_arm(controller_library)
    try:
        arm.torque_on_all()
        start = lib.fake_read(2, 116)
        now = time.monotonic()
        arm.command_q((-0.1, 1.0, 0.4, -0.4))
        time.sleep(0.08)
        elapsed = time.monotonic() - now
        limit = _native_config(HardwareConfig(), SimMappingConfig(), SafetyConfig()).max_motor_velocity_deg_s[1]
        change = abs(lib.fake_read(2, 116) - start)
        assert 0 < change <= limit * (elapsed + 0.025) * 4095 / 360 + 1
        # A stale queued target must not resume after torque is cycled.
        arm.torque_off_all()
        arm.torque_on_all()
        time.sleep(0.03)
        assert lib.fake_read(2, 116) == 2048
        config = _native_config(HardwareConfig(), SimMappingConfig(), SafetyConfig())
        profile_cap = math.floor(config.max_motor_velocity_deg_s[2] / (0.229 * 6.0))
        assert lib.fake_read(3, 112) == min(HardwareConfig().profile_vel_seg1, profile_cap)
    finally:
        arm.close()


def test_scheduler_stall_never_becomes_a_large_goal_jump(controller_library):
    lib = ctypes.CDLL(str(controller_library))
    lib.probe_stalled_step.argtypes = [ctypes.c_double]
    lib.probe_stalled_step.restype = ctypes.c_double
    assert lib.probe_stalled_step(10.0) == pytest.approx(0.2)
    assert lib.probe_stalled_step(0.005) == pytest.approx(0.05)


def test_reboot_faulted_motor_rebases_goal_then_verifies_all_torque_off(controller_library):
    from dataclasses import replace

    arm, lib = make_arm(
        controller_library,
        safety=replace(SafetyConfig(), read_failure_limit=1, monitor_period_s=0.01),
    )
    lib.fake_set_hardware_error.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.fake_reboot_count.argtypes = [ctypes.c_int]
    lib.fake_reboot_count.restype = ctypes.c_int
    try:
        arm.torque_on_all()
        lib.fake_set_position(5, 1777)
        lib.fake_set_hardware_error(5, 1)
        deadline = time.monotonic() + 1.0
        fault = arm.snapshot().fault
        while not fault and time.monotonic() < deadline:
            time.sleep(0.01)
            fault = arm.snapshot().fault
        assert "Dynamixel ID 5" in fault and "Hardware error occurred" in fault
        assert all(lib.fake_read(motor, 64) == 0 for motor in range(1, 6))

        assert arm.reboot_faulted_motor() == 5

        assert lib.fake_reboot_count(5) == 1
        assert lib.fake_read(5, 116) == 1777  # Old goal replaced before reboot.
        assert all(lib.fake_read(motor, 64) == 0 for motor in range(1, 6))
        state = arm.snapshot()
        assert state.fault == ""
        assert state.torque_enabled is False
    finally:
        arm.close()


def test_reboot_faulted_motor_requires_hardware_fault_and_torque_off(controller_library):
    arm, _lib = make_arm(controller_library)
    try:
        with pytest.raises(RuntimeError, match="no_rebootable_dynamixel_hardware_error"):
            arm.reboot_faulted_motor()
        arm.torque_on_all()
        with pytest.raises(RuntimeError, match="torque_must_be_off"):
            arm.reboot_faulted_motor()
    finally:
        arm.close()


def test_reboot_faulted_motor_checks_hardware_torque_registers(controller_library):
    from dataclasses import replace

    arm, lib = make_arm(
        controller_library,
        safety=replace(SafetyConfig(), read_failure_limit=1, monitor_period_s=0.01),
    )
    lib.fake_set_hardware_error.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.fake_set_torque.argtypes = [ctypes.c_int, ctypes.c_uint]
    try:
        lib.fake_set_hardware_error(5, 1)
        deadline = time.monotonic() + 1.0
        fault = arm.snapshot().fault
        while not fault and time.monotonic() < deadline:
            time.sleep(0.01)
            fault = arm.snapshot().fault
        assert "Hardware error occurred" in fault

        # Simulate a failed/externally overridden torque-off state while the
        # native software flag says off. Reboot must trust the register read.
        lib.fake_set_torque(2, 1)
        with pytest.raises(RuntimeError, match="torque_not_off_before_reboot_for_id_2"):
            arm.reboot_faulted_motor()
        assert lib.fake_read(5, 116) == 0  # No pre-reboot goal write occurred.
    finally:
        arm.close()


def test_gripper_contact_rebases_goal_without_dropping_arm_torque(controller_library):
    arm, lib = make_arm(controller_library)
    try:
        arm.torque_on_all()
        lib.fake_set_load_raw(5, 400)  # XL430 Present Load: 40% estimated load.
        arm.command_claw_deg(230.0)
        time.sleep(0.30)

        sample = arm.snapshot()
        assert sample.claw_load_permille == 400
        assert 5 not in sample.currents_ma

        measured_tick = lib.fake_read(5, 132)
        assert lib.fake_read(5, 116) == measured_tick
        assert all(lib.fake_read(motor, 64) == 1 for motor in range(1, 5))

        # Repeated close intent must not reapply the fully-closed goal after contact.
        arm.command_claw_deg(230.0)
        assert lib.fake_read(5, 116) == measured_tick

        # An explicit open command releases the contact hold.
        arm.command_claw_deg(340.0)
        assert lib.fake_read(5, 116) > measured_tick
    finally:
        arm.close()


def test_gripper_sustained_load_holds_while_still_compressing_object(controller_library):
    arm, lib = make_arm(controller_library)
    lib.fake_set_current_raw.argtypes = [ctypes.c_int, ctypes.c_uint]
    lib.fake_set_position.argtypes = [ctypes.c_int, ctypes.c_uint]
    stop = threading.Event()
    mover = None
    try:
        arm.torque_on_all()
        lib.fake_set_load_raw(5, 400)  # 40% estimated load, above claw threshold.
        arm.command_claw_deg(230.0)

        def compress_object():
            while not stop.is_set():
                measured = lib.fake_read(5, 132)
                goal = lib.fake_read(5, 116)
                if measured > goal:
                    lib.fake_set_position(5, max(goal, measured - 12))
                time.sleep(0.015)

        mover = threading.Thread(target=compress_object)
        mover.start()
        time.sleep(0.25)
        stop.set()
        mover.join(timeout=1.0)

        measured_tick = lib.fake_read(5, 132)
        assert measured_tick < 3000  # The gripper was still moving at contact.
        assert lib.fake_read(5, 116) == measured_tick
        assert all(lib.fake_read(motor, 64) == 1 for motor in range(1, 5))
    finally:
        stop.set()
        if mover is not None:
            mover.join(timeout=1.0)
        arm.close()
