#pragma once

#include <cstddef>
#include <cstdint>

// Stable C boundary used by the Robot DDS process. All motor I/O and the
// correction hook remain in native code; no Python callback runs in the loop.
extern "C" {

struct ElesimArmConfig {
  int32_t baudrate;
  int32_t motor_direction[4];
  int32_t profile_velocity[5];
  int32_t profile_acceleration[5];
  int32_t current_limit_ma;
  int32_t read_failure_limit;
  double monitor_period_s;
  double linear_motor_limit_deg;
  double q_min[4];
  double q_max[4];
  double motor_min_deg[4];
  double motor_max_deg[4];
};

struct ElesimArmSnapshot {
  double sampled_monotonic_s;
  int32_t ticks[5];
  int32_t currents_ma[5];
  int32_t torque_enabled;
  int32_t read_failures;
  int32_t valid;
};

void* elesim_arm_create(const char* device, const ElesimArmConfig* config,
                        char* error, size_t error_size);
int elesim_arm_open(void* handle, char* error, size_t error_size);
int elesim_arm_command_q(void* handle, const double* q,
                         char* error, size_t error_size);
int elesim_arm_command_claw(void* handle, double degrees,
                            char* error, size_t error_size);
int elesim_arm_torque_on(void* handle, char* error, size_t error_size);
int elesim_arm_torque_off(void* handle, char* error, size_t error_size);
int elesim_arm_safe_hold(void* handle, char* error, size_t error_size);
int elesim_arm_clear_fault(void* handle, char* error, size_t error_size);
int elesim_arm_snapshot(void* handle, ElesimArmSnapshot* snapshot,
                        char* fault, size_t fault_size);
int elesim_arm_close(void* handle, char* error, size_t error_size);
void elesim_arm_destroy(void* handle);

// Pure conversion probe used to check parity with the existing wire mapping.
int elesim_arm_map_q(const ElesimArmConfig* config, const double* q,
                     double* motor_degrees);

}
