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

Data lines have ten comma-separated fields:

```text
IMU1,time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0
IMU2,time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0
```

Boot, status, and command replies are also written to serial. Sensor reports
are requested every 50 ms; serial lines may be printed every 20 ms with the
last known quaternion. `time_ms` is the print time, not a fresh-sample
timestamp. A Robot reader must distinguish new sensor reports from repeated
lines before using sample age for safety.

The current Robot controller has a single three-component RPY input and no
Teensy serial reader. Connecting this sketch requires a bounded parser for
both IMU streams, unit/frame calibration, and an explicit six-component model
input decision. The shipped identity model works without IMU data.
