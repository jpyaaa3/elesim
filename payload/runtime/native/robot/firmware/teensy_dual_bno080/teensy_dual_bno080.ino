/*
 * Dual SparkFun BNO080/BNO085 reader for the 2-segment setup.
 *
 * Mapping:
 *   IMU1 = inner / upper arm = Wire1 / SDA17 / SCL16 / 0x4B
 *   IMU2 = outer / forearm   = Wire  / SDA18 / SCL19 / 0x4B
 *
 * Magnetometer use is intentionally fixed OFF.
 * Firmware always uses Game Rotation Vector.
 *
 * Serial protocol, 115200 baud:
 *   IMU1,time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0
 *   IMU2,time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,0
 */
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define IMU1_BUS Wire1
#define IMU1_SDA_PIN 17
#define IMU1_SCL_PIN 16
#define IMU1_ADDRESS 0x4B

#define IMU2_BUS Wire
#define IMU2_SDA_PIN 18
#define IMU2_SCL_PIN 19
#define IMU2_ADDRESS 0x4B

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t REPORT_PERIOD_MS = 50;   // 20 Hz sensor report
constexpr uint32_t PRINT_PERIOD_MS = 20;    // up to 50 Hz serial output
constexpr uint32_t STARTUP_DELAY_MS = 1000;

BNO080 imu1;
BNO080 imu2;

struct ImuState {
  float qw = 1.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  bool initialized = false;
  bool has_sample = false;
};

ImuState state1;
ImuState state2;

bool magnetometer_enabled = false;
bool imu_enabled = true;
String serial_command;

static float clampUnit(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

static float radiansToDegrees(float radians) {
  return radians * 57.29577951308232f;
}

static void quaternionToEuler(
    float qw, float qx, float qy, float qz,
    float &roll, float &pitch, float &yaw) {
  const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = radiansToDegrees(atan2f(sinr_cosp, cosr_cosp));

  const float sinp = 2.0f * (qw * qy - qz * qx);
  pitch = radiansToDegrees(asinf(clampUnit(sinp)));

  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = radiansToDegrees(atan2f(siny_cosp, cosy_cosp));
}

static void configureReports(BNO080 &imu) {
  imu.setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, 0);
  imu.enableGameRotationVector(REPORT_PERIOD_MS);
}

static void configureAllReports() {
  if (state1.initialized) configureReports(imu1);
  if (state2.initialized) configureReports(imu2);
}

static void setMagnetometerOff() {
  magnetometer_enabled = false;
  configureAllReports();
  Serial.println("MAGNETOMETER,OFF");
}

static void setImuEnabled(bool enabled) {
  imu_enabled = enabled;
  if (!imu_enabled) {
    state1.has_sample = false;
    state2.has_sample = false;
  }

  if (state1.initialized) {
    if (imu_enabled) {
      imu1.modeOn();
      configureReports(imu1);
    } else {
      imu1.modeSleep();
    }
  }

  if (state2.initialized) {
    if (imu_enabled) {
      imu2.modeOn();
      configureReports(imu2);
    } else {
      imu2.modeSleep();
    }
  }

  Serial.print("IMU_POWER,");
  Serial.println(imu_enabled ? "ON" : "OFF");
}

static void printStatus() {
  Serial.print("STATUS,IMU1=");
  Serial.print(state1.initialized ? "OK" : "FAIL");
  Serial.print(",IMU2=");
  Serial.print(state2.initialized ? "OK" : "FAIL");
  Serial.print(",MAG=OFF,IMU=");
  Serial.println(imu_enabled ? "ON" : "OFF");
}

static void handleCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (command == "MAG OFF" || command == "MAG=OFF" || command == "MAG 0") {
    setMagnetometerOff();
  } else if (command == "MAG ON" || command == "MAG=ON" || command == "MAG 1") {
    Serial.println("ERROR,MAGNETOMETER_FIXED_OFF");
    setMagnetometerOff();
  } else if (command == "IMU ON" || command == "IMU=ON") {
    setImuEnabled(true);
  } else if (command == "IMU OFF" || command == "IMU=OFF") {
    setImuEnabled(false);
  } else if (command == "MAG?" || command == "STATUS") {
    printStatus();
  } else if (command == "HELP") {
    Serial.println("COMMANDS,MAG OFF|IMU ON|IMU OFF|MAG?|STATUS|HELP");
  } else {
    Serial.print("ERROR,UNKNOWN_COMMAND,");
    Serial.println(command);
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n' || character == '\r') {
      handleCommand(serial_command);
      serial_command = "";
    } else if (serial_command.length() < 48) {
      serial_command += character;
    }
  }
}

static void updateSensor(BNO080 &imu, ImuState &state) {
  if (!imu_enabled || !state.initialized || !imu.dataAvailable()) {
    return;
  }
  state.qw = imu.getQuatReal();
  state.qx = imu.getQuatI();
  state.qy = imu.getQuatJ();
  state.qz = imu.getQuatK();
  state.has_sample = true;
}

static void printSensorLine(const char *label, const ImuState &state) {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  quaternionToEuler(state.qw, state.qx, state.qy, state.qz, roll, pitch, yaw);

  Serial.print(label);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(state.qw, 6);
  Serial.print(",");
  Serial.print(state.qx, 6);
  Serial.print(",");
  Serial.print(state.qy, 6);
  Serial.print(",");
  Serial.print(state.qz, 6);
  Serial.print(",");
  Serial.print(roll, 3);
  Serial.print(",");
  Serial.print(pitch, 3);
  Serial.print(",");
  Serial.print(yaw, 3);
  Serial.println(",0");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 4000) {
  }

  Serial.println("SPARKFUN_BNO080,READY");
  Serial.println(
      "PINOUT,IMU1=Wire1/SDA17/SCL16/ADDR0x4B,"
      "IMU2=Wire/SDA18/SCL19/ADDR0x4B"
  );

  // 작동하는 스케치와 동일한 초기화 순서: begin() 먼저, 그다음 핀 지정
  IMU1_BUS.begin();
  IMU1_BUS.setSCL(IMU1_SCL_PIN);
  IMU1_BUS.setSDA(IMU1_SDA_PIN);
  IMU1_BUS.setClock(400000);

  IMU2_BUS.begin();
  IMU2_BUS.setSCL(IMU2_SCL_PIN);
  IMU2_BUS.setSDA(IMU2_SDA_PIN);
  IMU2_BUS.setClock(400000);

  delay(STARTUP_DELAY_MS);

  Serial.print("INIT,IMU1,");
  state1.initialized = imu1.begin(IMU1_ADDRESS, IMU1_BUS);
  Serial.println(state1.initialized ? "OK" : "FAIL");
  if (state1.initialized) configureReports(imu1);

  Serial.print("INIT,IMU2,");
  state2.initialized = imu2.begin(IMU2_ADDRESS, IMU2_BUS);
  Serial.println(state2.initialized ? "OK" : "FAIL");
  if (state2.initialized) configureReports(imu2);

  setMagnetometerOff();
  printStatus();
  Serial.println(
      "DATA_FORMAT,IMU#,time_ms,qw,qx,qy,qz,"
      "roll_deg,pitch_deg,yaw_deg,mag_enabled"
  );
}

void loop() {
  pollSerialCommands();
  updateSensor(imu1, state1);
  updateSensor(imu2, state2);

  static uint32_t last_print = 0;
  const uint32_t now = millis();
  if (now - last_print >= PRINT_PERIOD_MS) {
    if (state1.has_sample) printSensorLine("IMU1", state1);
    if (state2.has_sample) printSensorLine("IMU2", state2);
    last_print = now;
  }
}
