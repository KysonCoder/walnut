#ifndef NIE_TARGET_HPP_
#define NIE_TARGET_HPP_

#include "armtarget.hpp"
#include "kinematic_error_info.hpp"

#include <array>

#include "legacy_robot_types.hpp"
#include "math/pose.hpp"
#include "math/se3_twist_ops.hpp"
#include "mdh_arm_kin.hpp"
#include "nie_arm_conf.hpp"

namespace interpolation {

/**
 * @brief NIE 单臂 6-DOF 运动学求解器。
 *
 * 平行于 CKinGeneRbt，操作 CArmTarget&（数据容器）。
 * 通过 arm.GetUnitType() 区分 LEFT_ARM / RIGHT_ARM。
 */
class CNieTarget {
 public:
  CNieTarget();

  /// FK：从 arm.joints[0..5] + waist_theta 计算世界系 pose，写回 arm.pose。
  /// @return 1 成功；ROBOT_TARGET_NOT_ATTACHED 未 Attach；
  ///         ROBOT_TARGET_POSE_CONFIG_INCOMPATABLE unit_type 非臂；
  ///         ROBOT_TARGET_NOT_INITIALIZED 关节数据无效。
  int CalcPose(CArmTarget& arm, double waist_theta);

  /// IK：从 arm.pose + waist_theta + 参考关节 ref 计算 joints[0..5]，写回 arm。
  /// @param err 详情通道（可选，nullptr 零开销）。2026-10-10 报告 §3.2。
  /// @return 1 成功；ROBOT_TARGET_POSE_OUTSIDE_REACH IK 无解；余同 CalcPose。
  int CalcJoint(CArmTarget& arm, double waist_theta, const CArmTarget& ref,
                KinematicErrorInfo* err = nullptr);

  /// Jacobian：6×6 几何法雅可比（世界系），行 [ωx,ωy,ωz,vx,vy,vz]。
  /// @return 同 CalcPose。
  int CalcVelocityJacobian(CArmTarget& arm, double waist_theta,
                           std::array<std::array<double, 6>, 6>& jacobian);

  void GetJointLimits(double jmin[NIE_PER_ARM], double jmax[NIE_PER_ARM]) const;
  void SetJointLimits(const double jmin[NIE_PER_ARM], const double jmax[NIE_PER_ARM]);

  /// 从 TargetData.kinematic_params 注入 MDH 参数与限位。
  /// @return true 已注入；false 参数为空（调用方保留 nie_arm_conf 缺省）。
  bool LoadParamsFromTargetData(const TargetData& data);
  void SetArmParams(const MdhArmParams& params);

  const MdhArmParams& GetArmParams() const { return arm_params_; }

 private:
  MdhArmParams arm_params_;
  double joint_min_[NIE_PER_ARM];
  double joint_max_[NIE_PER_ARM];

  static math::Pose<double> MatToPose_(const se3_twist::Mat4& T);
  static se3_twist::Mat4 PoseToMat_(const math::Pose<double>& pose);
  static void RotateZ_(double theta, double& x, double& y);

  static se3_twist::Mat4 WaistToArmBase_(SOC_UNIT_TYPE unit_type,
                                          double waist_theta);
  static se3_twist::Mat4 ArmBaseInverse_(SOC_UNIT_TYPE unit_type,
                                          double waist_theta);
};

}  // namespace interpolation

#endif  // NIE_TARGET_HPP_
