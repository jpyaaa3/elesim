#pragma once

#include <array>

namespace elesim_arm {

// This file is the future Teensy/IMU correction boundary. Today it is an
// identity transform: theoretical q is exactly the q sent to Dynamixel.
// ArmController calls this for the first goal and on every local control tick.
// A future implementation can read Teensy/IMU state and return a revised q
// without changing the DDS/runtime or Dynamixel output path. It must return
// promptly and never bypass ArmController's limits.
std::array<double, 4> correct_q(const std::array<double, 4>& theoretical_q);

}  // namespace elesim_arm
