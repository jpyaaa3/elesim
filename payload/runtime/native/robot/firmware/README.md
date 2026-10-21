# Teensy IMU firmware

`teensy_dual_bno080/teensy_dual_bno080.ino` is the source sketch for the two
BNO080/BNO085 sensors on the Robot arm. Flashing the Teensy is a separate
manual step; Robot installation does not program it.

The sketch uses the SparkFun BNO080 Arduino library, USB Serial, and Game
Rotation Vector reports with the magnetometer disabled. It calls
`Serial.begin(115200)`, but Teensy USB Serial transfers at USB speed and does
not use that baud rate for timing. `IMU1` is the
inner arm on Wire1 (SDA 17, SCL 16); `IMU2` is the outer arm on Wire (SDA 18,
SCL 19). Both sensors use I2C address `0x4B` on separate buses.

Current data lines have eleven comma-separated fields:

```text
IMU1,sample_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0,sequence
IMU2,sample_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0,sequence
```

Boot, status, and command replies are also written to serial. Sensor reports
are requested every 50 ms; serial lines may be printed every 20 ms with the
last known quaternion. `sample_ms` and the per-sensor uint32 `sequence` change
only when a sensor report arrives. Repeated lines cannot refresh reader age.
Older ten-field firmware used print time and no sequence. The reader accepts
it for diagnostics but marks freshness unverified, even if values are moving.

The POSIX Robot reader needs no extra Python dependency. In the Robot venv:

```bash
elesim-imu-read --device /dev/serial/by-id/<Teensy-device>
```

This diagnostic command prints JSON for both sensors and never opens the motor
port or DDS. Stop it before starting the Robot reader on the same device.
In the existing Robot YAML `runtime` section, set `imu_device` to the same path
to enable acquisition inside `elesim-robot`; empty (the default) disables it.
Use the actual device path on the Jetson, not the motor's `/dev/ttyUSB0`.
The service account needs serial-device read/write permission. Reader errors,
sensor values and age appear in the UI IMU Model panel through Robot telemetry.
Flash the updated sketch manually to get verified sequence-based freshness.

The native correction controller still has a single three-component RPY input.
Raw samples are diagnostic only until mounting/frame calibration and the
two-sensor model-input contract are defined. The identity model remains usable
without IMU data; reader connectivity does not mean correction is active.

## Full IMU CSV sketch

The reader also accepts the user-supplied `Dual BNO085 Full IMU Output` sketch
without reflashing to the rotation-only sketch above. It detects its 52-column
CSV automatically: `tx_ms,seq` followed by two 25-value sensor blocks:

```text
lastMcuMs,lastBnoUs,newAny,qw,qx,qy,qz,quatAcc,gravAcc,gyroAcc,
gvx,gvy,gvz,gx,gy,gz,lax,lay,laz,ax,ay,az,mx,my,mz
```

The displayed RPY is derived from the normalized quaternion, in radians. Other
vectors retain the SparkFun getters' units and sensor frames. Accuracy values
and MCU/BNO timestamps are retained in `full_imu` diagnostics. Serial framing
allows up to 2048 bytes per line, including six-decimal full reports.

`newAny=0` blocks never refresh the corresponding sensor's age, even if the
other sensor increments the transmit sequence. `newAny=1` means *any* report
changed; a gyro update may repeat the last quaternion. The UI therefore shows
recent/stale report reception separately from unverified orientation freshness.
An initial zero quaternion has no orientation. For verified quaternion age,
this sketch would need a rotation-specific sequence/timestamp per sensor.
Unlike the rotation-only sketch, Full IMU enables Rotation Vector and the
magnetometer; the reader sends no commands to change those settings.
