#pragma once

#include <array>
#include <cstdint>

namespace elesim_arm {

struct ImuSample {
  std::array<double, 3> rpy{};
  double sampled_monotonic_s = 0.0;
  bool valid = false;
};

struct Node {
  int32_t op = 0;
  int32_t a = 0;
  int32_t b = 0;
  double value = 0.0;
};

struct Program {
  int32_t count = 0;
  std::array<Node, 64> nodes{};
  std::array<int32_t, 4> outputs{};
  bool requires_imu = false;
};

// Pilot sends the model once. The fixed-size program runs without parsing or
// allocation on every local control tick.
Program identity_program();
Program compile_program(const Program& candidate);
std::array<double, 4> correct_q(const std::array<double, 4>& theoretical_q,
                                const ImuSample& imu, const Program& program);

}  // namespace elesim_arm
