#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <string>

// Register-backed SDK double: the production controller runs unchanged and
// no serial port or physical device can be opened by this test library.
#define COMM_SUCCESS 0
namespace dynamixel {
inline std::array<std::array<std::atomic<uint32_t>, 256>, 6> registers{};
inline std::array<std::atomic<uint8_t>, 6> hardware_errors{};
inline std::array<std::atomic<int>, 6> reboot_count{};
inline std::atomic<int> enabled_before_hold{0};
class PortHandler {
 public:
  static PortHandler* getPortHandler(const char*) { return new PortHandler; }
  bool openPort() { return true; }
  bool setBaudRate(int) { return true; }
  void closePort() {}
};
class PacketHandler {
 public:
  static PacketHandler* getPacketHandler(double) { static PacketHandler value; return &value; }
  const char* getTxRxResult(int) { return "stub"; }
  const char* getRxPacketError(uint8_t error) {
    return (error & 0x80) ? "[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!" : "stub";
  }
  int ping(PortHandler*, uint8_t, uint16_t* model, uint8_t* error) {
    *model = 1060; *error = 0; return 0;
  }
  int reboot(PortHandler*, uint8_t id, uint8_t* error) {
    *error = 0; ++reboot_count[id]; hardware_errors[id] = 0;
    registers[id][64] = 1;  // Model the shutdown-recovery torque restoration.
    return 0;
  }
  int write1ByteTxRx(PortHandler*, uint8_t id, uint16_t address, uint8_t value, uint8_t* error) {
    *error = hardware_errors[id].load() ? 0x80 : 0;
    if (address == 64 && value == 1 && registers[id][116] != registers[id][132])
      enabled_before_hold = 1;
    registers[id][address] = value;
    return 0;
  }
  int write4ByteTxRx(PortHandler*, uint8_t id, uint16_t address, uint32_t value, uint8_t* error) {
    *error = hardware_errors[id].load() ? 0x80 : 0; registers[id][address] = value; return 0;
  }
  int read2ByteTxRx(PortHandler*, uint8_t id, uint16_t address, uint16_t* value, uint8_t* error) {
    *error = hardware_errors[id].load() ? 0x80 : 0; *value = static_cast<uint16_t>(registers[id][address]); return 0;
  }
  int read1ByteTxRx(PortHandler*, uint8_t id, uint16_t address, uint8_t* value, uint8_t* error) {
    *error = hardware_errors[id].load() ? 0x80 : 0;
    *value = static_cast<uint8_t>(registers[id][address]); return 0;
  }
  int read4ByteTxRx(PortHandler*, uint8_t id, uint16_t address, uint32_t* value, uint8_t* error) {
    *error = hardware_errors[id].load() ? 0x80 : 0;
    *value = registers[id][address]; return 0;
  }
};
class GroupSyncRead {
 public:
  GroupSyncRead(PortHandler*, PacketHandler*, int, int) {}
  void clearParam() {}
  bool addParam(uint8_t) { return true; }
  int txRxPacket() { return 0; }
  bool isAvailable(uint8_t, int, int) { return true; }
  uint32_t getData(uint8_t id, int address, int) { return registers[id][address]; }
};
class GroupSyncWrite {
  std::map<uint8_t, uint32_t> pending;
 public:
  GroupSyncWrite(PortHandler*, PacketHandler*, int, int) {}
  void clearParam() { pending.clear(); }
  bool addParam(uint8_t id, uint8_t* value) {
    pending[id] = uint32_t(value[0]) | uint32_t(value[1]) << 8 |
                  uint32_t(value[2]) << 16 | uint32_t(value[3]) << 24;
    return true;
  }
  int txPacket() {
    for (auto item : pending) registers[item.first][116] = item.second;
    return 0;
  }
};
}  // namespace dynamixel
