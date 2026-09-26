#include "control.h"
#include "correction_placeholder.h"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>

namespace {

constexpr std::array<uint8_t, 5> kIds{1, 2, 3, 4, 5};
constexpr uint16_t kTorqueEnable = 64;
constexpr uint16_t kOperatingMode = 11;
constexpr uint16_t kProfileAcceleration = 108;
constexpr uint16_t kProfileVelocity = 112;
constexpr uint16_t kGoalPosition = 116;
constexpr uint16_t kPresentCurrent = 126;
constexpr uint16_t kPresentPosition = 132;
constexpr int kTickMax = 4095;
constexpr double kCurrentUnitMa = 2.69;
constexpr auto kControlPeriod = std::chrono::milliseconds(5);

double monotonic_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  }
  return static_cast<double>(value.tv_sec) +
         static_cast<double>(value.tv_nsec) / 1e9;
}

void copy_error(const std::string& message, char* output, size_t size) {
  if (output == nullptr || size == 0) return;
  const size_t count = std::min(message.size(), size - 1);
  std::memcpy(output, message.data(), count);
  output[count] = '\0';
}

double clamp(double value, double low, double high) {
  return std::min(std::max(value, low), high);
}

void validate_config(const ElesimArmConfig& cfg) {
  if (cfg.baudrate <= 0 || cfg.current_limit_ma <= 0 ||
      cfg.read_failure_limit <= 0 || !std::isfinite(cfg.monitor_period_s) ||
      cfg.monitor_period_s < 0.0 || !std::isfinite(cfg.linear_motor_limit_deg) ||
      cfg.linear_motor_limit_deg <= 0.0) {
    throw std::invalid_argument("invalid arm configuration");
  }
  for (int i = 0; i < 4; ++i) {
    if ((cfg.motor_direction[i] != 1 && cfg.motor_direction[i] != -1) ||
        !std::isfinite(cfg.q_min[i]) || !std::isfinite(cfg.q_max[i]) ||
        !std::isfinite(cfg.motor_min_deg[i]) ||
        !std::isfinite(cfg.motor_max_deg[i]) ||
        cfg.q_min[i] >= cfg.q_max[i] ||
        cfg.motor_min_deg[i] >= cfg.motor_max_deg[i]) {
      throw std::invalid_argument("invalid arm axis mapping");
    }
  }
  for (int i = 0; i < 5; ++i) {
    if (cfg.profile_velocity[i] <= 0 || cfg.profile_acceleration[i] <= 0) {
      throw std::invalid_argument("invalid arm motor profile");
    }
  }
}

std::array<double, 4> map_q(const ElesimArmConfig& cfg,
                            const std::array<double, 4>& q) {
  std::array<double, 4> degrees{};
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(q[i]) || q[i] < cfg.q_min[i] || q[i] > cfg.q_max[i]) {
      throw std::invalid_argument("q_out_of_bounds");
    }
    const double t = i == 0
                         ? (cfg.q_max[i] - q[i]) /
                               (cfg.q_max[i] - cfg.q_min[i])
                         : (q[i] - cfg.q_min[i]) /
                               (cfg.q_max[i] - cfg.q_min[i]);
    degrees[i] = cfg.motor_min_deg[i] +
                 t * (cfg.motor_max_deg[i] - cfg.motor_min_deg[i]);
  }
  return degrees;
}

int tick_for_degrees(double degrees, int direction, double low, double high) {
  if (!std::isfinite(degrees)) throw std::invalid_argument("nonfinite motor goal");
  const double limited = clamp(clamp(degrees, low, high), 0.0, 360.0);
  // Match the former Python driver's round-to-even tick conversion.
  int tick = static_cast<int>(std::nearbyint(
      limited * (static_cast<double>(kTickMax) / 360.0)));
  tick = std::clamp(tick, 0, kTickMax);
  return direction < 0 ? kTickMax - tick : tick;
}

class ArmController {
 public:
  ArmController(std::string device, const ElesimArmConfig& config)
      : device_(std::move(device)),
        config_(config),
        port_(dynamixel::PortHandler::getPortHandler(device_.c_str())),
        packet_(dynamixel::PacketHandler::getPacketHandler(2.0)),
        writer_(port_.get(), packet_, kGoalPosition, 4),
        reader_(port_.get(), packet_, kPresentPosition, 4) {
    validate_config(config_);
    if (device_.empty() || !port_ || !packet_) {
      throw std::invalid_argument("Dynamixel device is unavailable");
    }
  }

  ~ArmController() {
    try {
      close();
    } catch (...) {
    }
  }

  void open() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (opened_) return;
    if (!port_->openPort() || !port_->setBaudRate(config_.baudrate)) {
      port_->closePort();
      throw std::runtime_error("failed to open Dynamixel bus at configured baudrate");
    }
    reader_.clearParam();
    for (uint8_t id : kIds) {
      if (!reader_.addParam(id)) {
        port_->closePort();
        throw std::runtime_error("failed to configure Dynamixel position read");
      }
    }
    opened_ = true;
    stop_monitor_ = false;
    monitor_ = std::thread([this] { monitor_loop(); });
  }

  void close() {
    stop_monitor_ = true;
    if (monitor_.joinable()) monitor_.join();
    std::lock_guard<std::mutex> guard(mutex_);
    if (opened_) {
      port_->closePort();
      opened_ = false;
    }
  }

  void command_q(const double* raw) {
    if (raw == nullptr) throw std::invalid_argument("missing q");
    std::array<double, 4> theoretical{};
    std::copy(raw, raw + 4, theoretical.begin());
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    require_healthy();
    // Send the first goal now. The local monitor thread revisits this same
    // theoretical target on every control tick until hold/torque-off.
    const auto corrected = elesim_arm::correct_q(theoretical);
    write_arm_degrees(map_q(config_, corrected));
    theoretical_q_ = theoretical;
    last_corrected_q_ = corrected;
  }

  void command_claw(double degrees) {
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    require_healthy();
    const auto tick = tick_for_degrees(degrees, 1, 230.0, 340.0);
    write4(kIds[4], kGoalPosition, static_cast<uint32_t>(tick));
  }

  void torque_on() {
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    require_healthy();
    try {
      torque_off_locked();
      for (uint8_t id : kIds) write1(id, kOperatingMode, 3);
      for (size_t index = 0; index < kIds.size(); ++index) {
        write4(kIds[index], kProfileVelocity,
               static_cast<uint32_t>(config_.profile_velocity[index]));
        write4(kIds[index], kProfileAcceleration,
               static_cast<uint32_t>(config_.profile_acceleration[index]));
      }
      for (uint8_t id : kIds) write1(id, kTorqueEnable, 1);
      torque_enabled_ = true;
    } catch (...) {
      try {
        torque_off_locked();
      } catch (...) {
      }
      throw;
    }
  }

  void torque_off() {
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    torque_off_locked();
  }

  void safe_hold() {
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    theoretical_q_.reset();
    last_corrected_q_.reset();
    if (!torque_enabled_) return;
    std::array<int32_t, 5> ticks{};
    read_positions(ticks);
    std::array<double, 4> degrees{};
    for (int i = 0; i < 4; ++i) {
      const int tick = std::clamp(ticks[i], 0, kTickMax);
      const int oriented = config_.motor_direction[i] < 0 ? kTickMax - tick : tick;
      degrees[i] = static_cast<double>(oriented) * 360.0 / kTickMax;
    }
    write_arm_degrees(degrees);
  }

  void clear_fault() {
    std::lock_guard<std::mutex> guard(mutex_);
    require_open();
    if (torque_enabled_) throw std::runtime_error("torque_must_be_off");
    fault_.clear();
    read_failures_ = 0;
    try {
      sample_locked();
      require_healthy();
    } catch (const std::exception& error) {
      fault_ = error.what();
      throw;
    }
  }

  ElesimArmSnapshot snapshot(std::string& fault) {
    std::lock_guard<std::mutex> guard(mutex_);
    auto result = snapshot_;
    result.torque_enabled = torque_enabled_ ? 1 : 0;
    result.read_failures = read_failures_;
    fault = fault_;
    return result;
  }

 private:
  void require_open() const {
    if (!opened_) throw std::runtime_error("Dynamixel bus is closed");
  }

  void require_healthy() const {
    if (!fault_.empty()) throw std::runtime_error("arm safety fault: " + fault_);
  }

  void check_result(int result, uint8_t error, uint8_t id) const {
    if (result != COMM_SUCCESS) {
      throw std::runtime_error("Dynamixel ID " + std::to_string(id) + ": " +
                               packet_->getTxRxResult(result));
    }
    if (error != 0) {
      throw std::runtime_error("Dynamixel ID " + std::to_string(id) + ": " +
                               packet_->getRxPacketError(error));
    }
  }

  void write1(uint8_t id, uint16_t address, uint8_t value) {
    uint8_t error = 0;
    const int result = packet_->write1ByteTxRx(port_.get(), id, address, value, &error);
    check_result(result, error, id);
  }

  void write4(uint8_t id, uint16_t address, uint32_t value) {
    uint8_t error = 0;
    const int result = packet_->write4ByteTxRx(port_.get(), id, address, value, &error);
    check_result(result, error, id);
  }

  void torque_off_locked() {
    theoretical_q_.reset();
    last_corrected_q_.reset();
    std::string first_error;
    for (uint8_t id : kIds) {
      try {
        write1(id, kTorqueEnable, 0);
      } catch (const std::exception& error) {
        if (first_error.empty()) first_error = error.what();
      }
    }
    torque_enabled_ = false;
    if (!first_error.empty()) throw std::runtime_error(first_error);
  }

  void write_arm_degrees(const std::array<double, 4>& degrees) {
    writer_.clearParam();
    std::array<std::array<uint8_t, 4>, 4> data{};
    for (int i = 0; i < 4; ++i) {
      const double high = i == 0
                              ? std::min(config_.linear_motor_limit_deg,
                                         config_.motor_max_deg[i])
                              : config_.motor_max_deg[i];
      const uint32_t tick = static_cast<uint32_t>(tick_for_degrees(
          degrees[i], config_.motor_direction[i], config_.motor_min_deg[i], high));
      for (int byte = 0; byte < 4; ++byte) {
        data[i][byte] = static_cast<uint8_t>((tick >> (8 * byte)) & 0xff);
      }
      if (!writer_.addParam(kIds[i], data[i].data())) {
        throw std::runtime_error("failed to prepare Dynamixel sync write");
      }
    }
    check_result(writer_.txPacket(), 0, 254);
  }

  void read_positions(std::array<int32_t, 5>& ticks) {
    check_result(reader_.txRxPacket(), 0, 254);
    for (size_t index = 0; index < kIds.size(); ++index) {
      if (!reader_.isAvailable(kIds[index], kPresentPosition, 4)) {
        throw std::runtime_error("missing Dynamixel position sample");
      }
      ticks[index] = static_cast<int32_t>(
          reader_.getData(kIds[index], kPresentPosition, 4));
    }
  }

  void sample_locked() {
    std::array<int32_t, 5> ticks{};
    read_positions(ticks);
    std::array<int32_t, 5> currents{};
    for (size_t index = 0; index < kIds.size(); ++index) {
      uint16_t raw = 0;
      uint8_t error = 0;
      const int result = packet_->read2ByteTxRx(
          port_.get(), kIds[index], kPresentCurrent, &raw, &error);
      check_result(result, error, kIds[index]);
      currents[index] = static_cast<int32_t>(
          std::lround(static_cast<int16_t>(raw) * kCurrentUnitMa));
    }
    snapshot_.sampled_monotonic_s = monotonic_seconds();
    std::copy(ticks.begin(), ticks.end(), snapshot_.ticks);
    std::copy(currents.begin(), currents.end(), snapshot_.currents_ma);
    snapshot_.valid = 1;
    read_failures_ = 0;
    for (size_t index = 0; index < kIds.size(); ++index) {
      if (std::abs(currents[index]) > config_.current_limit_ma) {
        trip_fault_locked("motor current limit exceeded: ID " +
                          std::to_string(kIds[index]));
        break;
      }
    }
  }

  void trip_fault_locked(const std::string& reason) {
    if (!fault_.empty()) return;
    fault_ = reason;
    std::cerr << "[robot-arm] safety fault latched: " << fault_ << std::endl;
    try {
      torque_off_locked();
    } catch (const std::exception& error) {
      fault_ += "; torque disable failed: ";
      fault_ += error.what();
      std::cerr << "[robot-arm] " << fault_ << std::endl;
    }
  }

  void monitor_loop() {
    const auto telemetry_period = std::chrono::duration<double>(config_.monitor_period_s);
    auto next_telemetry = std::chrono::steady_clock::now();
    while (!stop_monitor_) {
      std::this_thread::sleep_for(kControlPeriod);
      if (stop_monitor_) break;
      std::lock_guard<std::mutex> guard(mutex_);
      if (!opened_ || !fault_.empty()) continue;
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_telemetry) {
        next_telemetry = now +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                telemetry_period);
        try {
          sample_locked();
        } catch (const std::exception& error) {
          ++read_failures_;
          if (read_failures_ >= config_.read_failure_limit) {
            trip_fault_locked("hardware telemetry unavailable after " +
                              std::to_string(read_failures_) +
                              " reads: " + error.what());
          }
          continue;
        }
      }
      if (!fault_.empty() || !torque_enabled_ || !theoretical_q_) continue;
      try {
        const auto corrected = elesim_arm::correct_q(*theoretical_q_);
        if (last_corrected_q_ != corrected) {
          write_arm_degrees(map_q(config_, corrected));
          last_corrected_q_ = corrected;
        }
      } catch (const std::exception& error) {
        trip_fault_locked(std::string("local control failed: ") + error.what());
      }
    }
  }

  std::string device_;
  ElesimArmConfig config_;
  std::unique_ptr<dynamixel::PortHandler> port_;
  dynamixel::PacketHandler* packet_;
  dynamixel::GroupSyncWrite writer_;
  dynamixel::GroupSyncRead reader_;
  std::mutex mutex_;
  std::thread monitor_;
  std::atomic<bool> stop_monitor_{false};
  bool opened_ = false;
  bool torque_enabled_ = false;
  std::string fault_;
  int read_failures_ = 0;
  ElesimArmSnapshot snapshot_{};
  std::optional<std::array<double, 4>> theoretical_q_;
  std::optional<std::array<double, 4>> last_corrected_q_;
};

template <typename Function>
int guarded(Function&& function, char* error, size_t error_size) {
  try {
    function();
    copy_error("", error, error_size);
    return 0;
  } catch (const std::exception& caught) {
    copy_error(caught.what(), error, error_size);
    return -1;
  } catch (...) {
    copy_error("unknown native arm failure", error, error_size);
    return -1;
  }
}

ArmController& arm(void* handle) {
  if (!handle) throw std::invalid_argument("null arm controller");
  return *static_cast<ArmController*>(handle);
}

}  // namespace

extern "C" {

void* elesim_arm_create(const char* device, const ElesimArmConfig* config,
                        char* error, size_t error_size) {
  ArmController* result = nullptr;
  const int status = guarded(
      [&] {
        if (!device || !config) throw std::invalid_argument("missing arm config");
        result = new ArmController(device, *config);
      },
      error, error_size);
  return status == 0 ? result : nullptr;
}

int elesim_arm_open(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).open(); }, error, size);
}
int elesim_arm_command_q(void* handle, const double* q, char* error, size_t size) {
  return guarded([&] { arm(handle).command_q(q); }, error, size);
}
int elesim_arm_command_claw(void* handle, double degrees, char* error, size_t size) {
  return guarded([&] { arm(handle).command_claw(degrees); }, error, size);
}
int elesim_arm_torque_on(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).torque_on(); }, error, size);
}
int elesim_arm_torque_off(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).torque_off(); }, error, size);
}
int elesim_arm_safe_hold(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).safe_hold(); }, error, size);
}
int elesim_arm_clear_fault(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).clear_fault(); }, error, size);
}
int elesim_arm_snapshot(void* handle, ElesimArmSnapshot* snapshot,
                        char* fault, size_t fault_size) {
  try {
    if (!snapshot) throw std::invalid_argument("missing snapshot output");
    std::string fault_value;
    *snapshot = arm(handle).snapshot(fault_value);
    copy_error(fault_value, fault, fault_size);
    return 0;
  } catch (const std::exception& error) {
    copy_error(error.what(), fault, fault_size);
    return -1;
  }
}
int elesim_arm_close(void* handle, char* error, size_t size) {
  return guarded([&] { arm(handle).close(); }, error, size);
}
void elesim_arm_destroy(void* handle) {
  delete static_cast<ArmController*>(handle);
}
int elesim_arm_map_q(const ElesimArmConfig* config, const double* q,
                     double* motor_degrees) {
  if (!config || !q || !motor_degrees) return -1;
  try {
    std::array<double, 4> input{};
    std::copy(q, q + 4, input.begin());
    const auto mapped = map_q(*config, elesim_arm::correct_q(input));
    std::copy(mapped.begin(), mapped.end(), motor_degrees);
    return 0;
  } catch (...) {
    return -1;
  }
}

}
