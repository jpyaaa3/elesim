"""Real PTY acquisition plus malformed/stale firmware report regressions."""
import math
import os
import pty
import time
import unittest

from elesim_robot.imu.teensy import TeensyReader, parse_line, parse_full_line


def report(sensor="IMU1", seq=1):
    suffix = "" if seq is None else f",{seq}"
    return f"{sensor},100,1,0,0,0,90,0,-90,0{suffix}\n".encode()


def full_report(seq=1, new1=1, new2=1, quat1=None, quat2=None):
    fields = [120, seq]
    for index, (new, quat) in enumerate(((new1, quat1), (new2, quat2))):
        # Exactly the 25 values from the user's printImu(), including all vectors.
        fields += [100 + index, 50000 + index, new]
        fields += [1, 0, 0, 0] if quat is None else quat
        fields += [3, 2, 1]
        fields += [index * 100 + v / 10 for v in range(15)]
    assert len(fields) == 52
    return (",".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in fields) + "\n").encode()


class TeensyTests(unittest.TestCase):
    def test_full_csv_two_sensor_field_mapping_and_rpy(self):
        q = math.sqrt(.5)
        samples = parse_full_line(full_report(quat1=[q, q, 0, 0]), 2.0)
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[0].rpy_rad[0], math.pi / 2)
        self.assertEqual(samples[1].full_imu["bno_time_us"], 50001)
        self.assertEqual(samples[1].full_imu["accuracy"], {"quat": 3, "gravity": 2, "gyro": 1})
        self.assertEqual(samples[0].full_imu["gravity"], [0, .1, .2])
        self.assertEqual(samples[0].full_imu["gyro"], [.3, .4, .5])
        self.assertEqual(samples[1].full_imu["mag"], [101.2, 101.3, 101.4])

    def test_full_csv_new_any_does_not_prove_rotation_freshness(self):
        now = [1.0]
        reader = TeensyReader("/dev/unused", clock=lambda: now[0])
        reader._connected = True
        reader.feed(full_report())
        now[0] += .3
        reader.feed(full_report(seq=2, new1=1, new2=0))
        samples = reader.snapshot()["sensors"]
        self.assertTrue(samples["IMU1"]["reports_recent"])
        self.assertFalse(samples["IMU1"]["fresh"])
        self.assertFalse(samples["IMU1"]["freshness_verified"])
        self.assertFalse(samples["IMU2"]["reports_recent"])
        self.assertEqual(samples["IMU2"]["sequence"], 1)
        now[0] += .3
        reader.feed(full_report(seq=2))  # duplicate cannot keep IMU1 alive
        self.assertFalse(reader.snapshot()["sensors"]["IMU1"]["reports_recent"])

    def test_full_csv_header_fragmentation_initial_zero_and_reboot(self):
        reader = TeensyReader("/dev/unused")
        reader.feed(b"tx_ms,seq,t1_mcu_ms,...\n")
        line = full_report(seq=9, quat1=[0, 0, 0, 0])
        self.assertGreater(len(line), 256)
        reader.feed(line[:200])
        reader.feed(line[200:])
        samples = reader.snapshot()["sensors"]
        self.assertIsNone(samples["IMU1"]["rpy_rad"])
        self.assertIsNotNone(samples["IMU2"]["rpy_rad"])
        reader.feed(b"=== Dual BNO085 Full IMU Output ===\n" + full_report(seq=1))
        self.assertEqual(reader.snapshot()["sensors"]["IMU1"]["sequence"], 1)

    def test_full_csv_rejects_bad_count_flags_and_nonfinite_vectors(self):
        for line in (b"120,1\n", full_report(new1=2),
                     full_report().replace(b"0.1", b"nan", 1),
                     full_report(seq=-1), full_report(quat1=[2, 0, 0, 0])):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_full_line(line, 1)

    def test_units_and_legacy(self):
        sample = parse_line(report(), 2.0)
        self.assertEqual(sample.quaternion_wxyz, (1, 0, 0, 0))
        self.assertAlmostEqual(sample.rpy_rad[0], math.pi / 2)
        self.assertIsNone(parse_line(b"STATUS,IMU1=OK\n", 2))
        self.assertIsNone(parse_line(report(seq=None), 2).sequence)

    def test_corrupt_reports(self):
        for line in (b"IMU1,garbage", report().replace(b",1,0,0,0,", b",nan,0,0,0,"),
                     report().replace(b",1,0,0,0,", b",0,0,0,0,"),
                     report(seq=-1), b"IMU1," + b"x" * 300):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_line(line, 1)

    def test_split_lines_bounded_recovery_and_independent_streams(self):
        reader = TeensyReader("/dev/unused")
        line = report()
        reader.feed(line[:9])
        self.assertEqual(reader.snapshot()["sensors"], {})
        reader.feed(line[9:] + b"x" * 20000 + b"\n" + report("IMU2"))
        state = reader.snapshot()
        self.assertEqual(set(state["sensors"]), {"IMU1", "IMU2"})
        self.assertEqual(state["invalid_lines"], 1)
        self.assertEqual(len(reader._buffer), 0)

    def test_duplicate_backward_wrap_reboot_and_legacy_freshness(self):
        now = [1.0]
        reader = TeensyReader("/dev/unused", clock=lambda: now[0])
        reader._connected = True
        reader.feed(report(seq=2**32 - 1))
        now[0] += .05
        reader.feed(report(seq=0))  # valid uint32 wrap
        self.assertTrue(reader.snapshot()["sensors"]["IMU1"]["fresh"])
        now[0] += .3
        reader.feed(report(seq=0) + report(seq=2**32 - 1))
        self.assertFalse(reader.snapshot()["sensors"]["IMU1"]["fresh"])
        reader.feed(b"SPARKFUN_BNO080,READY\n" + report(seq=1))
        self.assertTrue(reader.snapshot()["sensors"]["IMU1"]["fresh"])
        reader.feed(report("IMU2", seq=None))
        self.assertFalse(reader.snapshot()["sensors"]["IMU2"]["fresh"])

    def test_actual_serial_pty_and_shutdown(self):
        master, slave = pty.openpty()
        reader = TeensyReader(os.ttyname(slave))
        try:
            reader.start()
            deadline = time.monotonic() + 2
            while not reader.snapshot()["connected"] and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(reader.snapshot()["connected"])
            os.write(master, report() + report("IMU2"))
            while len(reader.snapshot()["sensors"]) < 2 and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(len(reader.snapshot()["sensors"]), 2)
            self.assertTrue(reader.snapshot()["sensors"]["IMU1"]["fresh"])
            os.write(master, full_report(seq=2))
            while (reader.snapshot()["sensors"]["IMU1"]["source_format"] != "full-imu-v1"
                   and time.monotonic() < deadline):
                time.sleep(.01)
            sample = reader.snapshot()["sensors"]["IMU1"]
            self.assertEqual(sample["source_format"], "full-imu-v1")
            self.assertTrue(sample["reports_recent"])
            self.assertFalse(sample["fresh"])
            os.close(master)
            master = None
            deadline = time.monotonic() + 2
            while reader.snapshot()["connected"] and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(reader.snapshot()["connected"])
        finally:
            reader.stop()
            if master is not None:
                os.close(master)
            os.close(slave)

    def test_missing_device_stops_during_retry(self):
        reader = TeensyReader("/dev/elesim-test-missing")
        reader.start()
        try:
            deadline = time.monotonic() + 2
            while not reader.snapshot()["error"] and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(reader.snapshot()["connected"])
            self.assertTrue(reader.snapshot()["error"])
        finally:
            reader.stop()

    def test_runtime_reader_lifecycle_and_wire_diagnostics_do_not_enable_correction(self):
        from elesim_robot.config import HardwareConfig
        from elesim_robot.runtime import RobotRuntime
        from elesim_protocol import SimMappingConfig, make_envelope
        from elesim_protocol.messages import dumps_envelope, loads_envelope
        runtime = RobotRuntime(mapping=SimMappingConfig(), hardware_config=HardwareConfig(),
                               imu_device="/dev/elesim-test-missing")
        try:
            runtime.open()
            runtime.imu_reader.feed(full_report())
            state = runtime.state()
            self.assertFalse(state["imu_feedback_connected"])
            wire = dumps_envelope(make_envelope("telemetry", "robot-a", payload=state))
            restored = loads_envelope(wire).payload
            self.assertEqual(set(restored["imu_reader"]["sensors"]), {"IMU1", "IMU2"})
            self.assertEqual(restored["imu_reader"]["sensors"]["IMU2"]["full_imu"]["gyro"],
                             [100.3, 100.4, 100.5])
        finally:
            runtime.close()
        self.assertIsNone(runtime.imu_reader._thread)


if __name__ == "__main__":
    unittest.main()
