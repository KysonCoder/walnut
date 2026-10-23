#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CALCULATOR_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CALCULATOR_HPP_

#include "drivechain/drive_chain_types.hpp"
#include "drivechain/drive_chain_config.hpp"
#include "drivechain/drive_chain_tables.hpp"

namespace drivechain {

class DriveChainCalculator {
 public:
  bool BuildTables(const DriveChainJointConfig& config,
                   DriveChainJointTables* tables);

  bool GetTorque(const DriveChainJointTables& tables,
                 float speed,
                 bool is_acceleration,
                 DriveChainMode mode,
                 float* torque) const;

  bool GetEmergencyTorque(const DriveChainJointTables& tables,
                          float speed,
                          EmergencyPhase phase,
                          float* torque_max,
                          float* torque_min) const;

  void GetTorqueBatch(const DriveChainJointTables* tables_array,
                      int joint_count,
                      const float* speeds,
                      const bool* is_accel_flags,
                      DriveChainMode mode,
                      float* torques_out) const;

  static float JointSpeedToMotor(float joint_speed, float transmission_ratio);
  static float MotorTorqueToJoint(float motor_torque, float transmission_ratio);

  // Exposed for testing
  float CalcFinalTorque(const DriveChainJointConfig& config,
                        float speed, float acc_sign) const;

 private:
  bool ValidateConfig(const DriveChainJointConfig& config) const;

  float CalcThermalTorque(const DriveChainJointConfig& config,
                          float speed, float acc_sign) const;
  float CalcStressTorque(const DriveChainJointConfig& config,
                         float speed, float acc_sign) const;
  float CalcElectricalTorque(const DriveChainJointConfig& config,
                             float speed) const;
  float CalcFriction(const DriveChainJointConfig& config,
                     float speed, bool use_max) const;
  float CalcEmergencyTorqueAtSpeed(const DriveChainJointConfig& config,
                                   float speed, EmergencyPhase phase) const;

  bool InsertTable(DriveChainTablePoint* table, int index,
                   float delta_speed, float prev_torque, float torque);
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_CALCULATOR_HPP_
