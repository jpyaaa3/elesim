import ctypes
import shutil
import subprocess
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
extern "C" void fake_reset() {
  dynamixel::enabled_before_hold = 0;
  for (int id=1; id<=5; ++id) {
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


def make_arm(path):
    arm = NativeArm("test-only-no-hardware", hardware=HardwareConfig(), mapping=SimMappingConfig(),
                    safety=SafetyConfig(), library_path=path)
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
        assert lib.fake_read(3, 112) <= 21  # firmware cap also follows the physical rate.
    finally:
        arm.close()


def test_scheduler_stall_never_becomes_a_large_goal_jump(controller_library):
    lib = ctypes.CDLL(str(controller_library))
    lib.probe_stalled_step.argtypes = [ctypes.c_double]
    lib.probe_stalled_step.restype = ctypes.c_double
    assert lib.probe_stalled_step(10.0) == pytest.approx(0.2)
    assert lib.probe_stalled_step(0.005) == pytest.approx(0.05)
