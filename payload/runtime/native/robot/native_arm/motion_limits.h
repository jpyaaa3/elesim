#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace elesim_arm {

// Never catch up a stalled thread with one large physical goal jump.
inline std::array<double, 4> limit_motor_step(
    const std::array<double, 4>& current,
    const std::array<double, 4>& target,
    const double* max_velocity, double elapsed_s) {
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0)
    throw std::invalid_argument("invalid control interval");
  const double dt = std::min(elapsed_s, 0.02);
  std::array<double, 4> next{};
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(current[i]) || !std::isfinite(target[i]) ||
        !std::isfinite(max_velocity[i]) || max_velocity[i] <= 0.0)
      throw std::invalid_argument("invalid motor slew limit");
    const double step = max_velocity[i] * dt;
    next[i] = std::clamp(target[i], current[i] - step, current[i] + step);
  }
  return next;
}

}  // namespace elesim_arm
