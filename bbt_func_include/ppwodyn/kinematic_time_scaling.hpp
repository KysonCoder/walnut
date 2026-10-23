#ifndef FUNC_PPWODYN_INCLUDE_KINEMATIC_TIME_SCALING_HPP_
#define FUNC_PPWODYN_INCLUDE_KINEMATIC_TIME_SCALING_HPP_

#include "kinematic_topp_profiler.hpp"
#include "ppwodyn_config.hpp"

namespace ppwodyn {

// 运动学版时间缩放可行性（替换 action::TimeScalingFeasibility 的动力学力矩检查）。
// 均匀时间缩放 α = T_old/T_new：速度×α、加速度×α²。
// α 的上界由关节/笛卡尔的速度与加速度限幅共同决定。
class KinematicTimeScaling {
 public:
  // 计算 profile 均匀缩放到 target_duration_s 所需 α=T_old/T_new 是否可行，
  // 并输出该 profile 允许的最大 α（α_max >= 1 表示有加速余量）。
  // profile 不可行 / 尺寸不一致 / 目标时长非法 → false。
  bool ComputeMaxScale(const KinematicToppProfiler::Profile& profile,
                       double target_duration_s,
                       const KinematicLimits& limits,
                       double& alpha_out) const;
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_KINEMATIC_TIME_SCALING_HPP_
