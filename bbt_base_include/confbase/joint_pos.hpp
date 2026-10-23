#ifndef BASE_CONFBASE_INCLUDE_JOINT_POS_HPP_
#define BASE_CONFBASE_INCLUDE_JOINT_POS_HPP_

#include "shm_layout.hpp"

class CJointPos {
 public:
  // No default constructor — avoids implicit heap allocation
  CJointPos() = delete;

  // Attach to external data (zero-copy)
  explicit CJointPos(double* external_data,
                     const robot_config::GlobalJointMap* map = nullptr,
                     const robot_config::GlobalJointParams* params = nullptr);

  // No destructor — does not own data

  CJointPos(const CJointPos&) = delete;
  CJointPos& operator=(const CJointPos&) = delete;
  CJointPos(CJointPos&&) = default;
  CJointPos& operator=(CJointPos&&) = default;

  double Get(int index) const;
  void Set(int index, double value);

  double GetLogical(int logical_idx) const;
  void SetLogical(int logical_idx, double value);
  double GetPhysical(int physical_idx) const;
  void SetPhysical(int physical_idx, double value);

  double GetUnitJoint(int unit_local_idx,
                      const robot_config::UnitConfig* unit) const;
  void SetUnitJoint(int unit_local_idx, double value,
                    const robot_config::UnitConfig* unit);

  void JointPosToMotorPos(CJointPos* motor_pos) const;
  void MotorPosToJointPos(const CJointPos* motor_pos);
  void EncoderPosToMotorPos(CJointPos* motor_pos) const;
  void MotorPosToEncoderPos(CJointPos* encoder_pulse) const;

  void JointPosToMotorPos(const robot_config::UnitConfig* unit,
                          CJointPos* motor_pos) const;
  void MotorPosToJointPos(const robot_config::UnitConfig* unit,
                          CJointPos* joint_pos);

  void Reset();
  bool IsAttached() const { return data_ != nullptr; }

 private:
  double* data_ = nullptr;  // Not owned, just referenced
  const robot_config::GlobalJointMap* map_ = nullptr;
  const robot_config::GlobalJointParams* params_ = nullptr;
};

#endif  // BASE_CONFBASE_INCLUDE_JOINT_POS_HPP_
