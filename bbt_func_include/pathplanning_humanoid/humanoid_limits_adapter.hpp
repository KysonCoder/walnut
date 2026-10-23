// func/pathplanning_humanoid/include/humanoid_limits_adapter.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_LIMITS_ADAPTER_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_LIMITS_ADAPTER_HPP_

#include "confbase/humanoid_config_store.hpp"
#include "pphumanoid_types.hpp"

namespace pphumanoid {

// drivechain 热/应力表 → 19-DOF 力矩上限 + 关节速度限制。
// 索引映射：ROBOT_BODY→0..3, ROBOT_HEAD→4, LEFT_ARM→5..11, RIGHT_ARM→12..18
// （与 dyn_humanoid 全身布局 kChainOffset 严格一致）。
// 读取模式参照老库 PathPlanningLimitsAdapter::FillTorqueLimits13。
class HumanoidLimitsAdapter {
 public:
  HumanoidLimitsAdapter() = default;

  bool Attach(const robot_config::HumanoidConfigStore& store);
  bool IsAttached() const { return attached_; }

  // 填充 tau_max/tau_min。已有覆盖的索引段跳过表读取。
  // 任一索引既无表数据又无覆盖 → PPH_ERR_TORQUE_LIMITS_MISSING（部分填充仍生效）。
  int FillTorqueLimits(dyn_humanoid::VecW& tau_max,
                       dyn_humanoid::VecW& tau_min) const;

  // 关节速度上限 [rad/s]，来自 confbase 单元 joints_drive.speed_positive_limit。
  int FillJointVelocityLimits(dyn_humanoid::VecW& qd_max) const;

  // 局部覆盖：[begin, end) 索引段。SetTorqueOverride(0,19,...) 即整体覆盖。
  void SetTorqueOverride(int begin, int end, const dyn_humanoid::VecW& tau_max,
                         const dyn_humanoid::VecW& tau_min);
  void ClearTorqueOverride();

 private:
  bool attached_ = false;
  bool override_mask_[dyn_humanoid::kWholeBodyDof] = {};
  dyn_humanoid::VecW override_max_{};
  dyn_humanoid::VecW override_min_{};
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_LIMITS_ADAPTER_HPP_
