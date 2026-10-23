#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CONFIG_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CONFIG_HPP_

#include <cstdint>

namespace drivechain {

struct DriveChainJointConfig {
  // Motor parameters
  float motor_torque_0;
  float motor_ke;
  float motor_i_n;
  float motor_i_max;
  float motor_speed_max;
  float motor_torque_losses[3];
  float motor_speeds[3];

  // Thermal duty cycle
  float thermal_torque_turbo_acc;
  float thermal_torque_turbo_dec;
  float thermal_torque_max_acc;
  float thermal_torque_max_dec;
  float thermal_speed_turbo;
  float thermal_speed_max;
  float thermal_torque_static;
  float thermal_int_turbo_acc;
  float thermal_int_turbo_dec;
  float thermal_int_max_acc;
  float thermal_int_max_dec;

  // Stress duty cycle
  float stress_torque_turbo_acc;
  float stress_torque_turbo_dec;
  float stress_torque_max_acc;
  float stress_torque_max_dec;
  float stress_speed_turbo;
  float stress_speed_max;
  float stress_speed_absolute_max;
  float stress_torque_absolute_max;

  // Gearbox friction (4-point piecewise linear)
  float gear_max_fric[4];
  float gear_min_fric[4];
  float gear_fric_speeds[3];

  // Transmission
  float transmission_ratio;
  bool rotating_move;

  // Brake (3-point piecewise linear)
  float brake_max_torque[3];
  float brake_min_torque[3];
  float brake_speeds[3];

  // Emergency stop
  float em_stop_torque_turbo;
  float em_stop_torque_max;
  float em_stop_speed_max;
  float em_stop_speed_turbo;
  float em_stop_torque_min_max_quota;

  // Simplified electrical limits
  float drive_i_n;
  float drive_i_max;
  float dc_bus_voltage;
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CONFIG_HPP_
