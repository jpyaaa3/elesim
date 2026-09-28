#include "correction_model.h"

#include <cmath>
#include <stdexcept>

namespace elesim_arm {

Program identity_program() {
  Program result{};
  result.count = 4;
  for (int i = 0; i < 4; ++i) {
    result.nodes[i] = Node{1, i, 0, 0.0};
    result.outputs[i] = i;
  }
  return result;
}

Program compile_program(const Program& candidate) {
  if (candidate.count < 4 || candidate.count > 64) {
    throw std::invalid_argument("IMU model needs 4..64 nodes");
  }
  auto result = candidate;
  result.requires_imu = false;
  for (int i = 0; i < result.count; ++i) {
    const auto& node = result.nodes[i];
    switch (node.op) {
      case 1:
        if (node.a < 0 || node.a > 3) throw std::invalid_argument("invalid q index");
        break;
      case 2:
        if (node.a < 0 || node.a > 2) throw std::invalid_argument("invalid IMU index");
        result.requires_imu = true;
        break;
      case 3:
        if (!std::isfinite(node.value) || std::abs(node.value) > 1e6) {
          throw std::invalid_argument("invalid IMU model constant");
        }
        break;
      case 4:
      case 5:
      case 6:
      case 7:
        if (node.a < 0 || node.a >= i || node.b < 0 || node.b >= i) {
          throw std::invalid_argument("invalid binary node reference");
        }
        break;
      case 8:
      case 9:
      case 10:
        if (node.a < 0 || node.a >= i) {
          throw std::invalid_argument("invalid unary node reference");
        }
        break;
      default:
        throw std::invalid_argument("unsupported IMU model operation");
    }
  }
  for (int output : result.outputs) {
    if (output < 0 || output >= result.count) {
      throw std::invalid_argument("invalid IMU model output");
    }
  }
  return result;
}

std::array<double, 4> correct_q(const std::array<double, 4>& theoretical_q,
                                const ImuSample& imu, const Program& program) {
  if (program.requires_imu && !imu.valid) {
    throw std::runtime_error("IMU sample unavailable or stale");
  }
  std::array<double, 64> values{};
  for (int i = 0; i < program.count; ++i) {
    const auto& node = program.nodes[i];
    double value = 0.0;
    switch (node.op) {
      case 1: value = theoretical_q[node.a]; break;
      case 2: value = imu.rpy[node.a]; break;
      case 3: value = node.value; break;
      case 4: value = values[node.a] + values[node.b]; break;
      case 5: value = values[node.a] - values[node.b]; break;
      case 6: value = values[node.a] * values[node.b]; break;
      case 7:
        if (std::abs(values[node.b]) < 1e-12) {
          throw std::runtime_error("IMU model division by zero");
        }
        value = values[node.a] / values[node.b];
        break;
      case 8: value = -values[node.a]; break;
      case 9: value = std::sin(values[node.a]); break;
      case 10: value = std::cos(values[node.a]); break;
      default: throw std::runtime_error("invalid compiled IMU model");
    }
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite IMU model result");
    values[i] = value;
  }
  std::array<double, 4> corrected{};
  for (int i = 0; i < 4; ++i) corrected[i] = values[program.outputs[i]];
  return corrected;
}

}  // namespace elesim_arm
