// func/pathplanning_humanoid/include/humanoid_topp_profiler.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TOPP_PROFILER_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TOPP_PROFILER_HPP_

#include "humanoid_zmp_constraint.hpp"
#include "pphumanoid_types.hpp"

namespace dyn_humanoid {
class HumanoidDynFacade;
}

namespace pphumanoid {

// 全身耦合 TOPP：对共享路径参数 s∈[0,1] 的 19-DOF q(s) 做时间最优
// 参数化。约束：力矩（dyn_humanoid AccelBounds）∩ ZMP（可选）∩ 速度上限。
// 不可行时回退梯形速度曲线（used_fallback=true），规划永远有结果。
class HumanoidToppProfiler {
 public:
  struct Config {
    double fallback_v_max_cart = 0.1;   // 回退梯形速度 [m/s]
    double fallback_a_max_cart = 0.5;   // 回退梯形加速度 [m/s^2]
  };

  HumanoidToppProfiler();
  explicit HumanoidToppProfiler(const Config& config);

  // path: 全身采样表（num_samples >= 2）。zmp 为 nullptr 时跳过 ZMP 约束。
  // 返回 PPH_OK 表示产生了可用 profile（含回退）；
  // PPH_ERR_TOPP_INFEASIBLE 仅在连回退都无法构造时返回（如 ref 长度为 0）。
  int Plan(const WholeBodyPathSamples& path,
           const dyn_humanoid::HumanoidDynFacade& facade,
           const dyn_humanoid::VecW& tau_max, const dyn_humanoid::VecW& tau_min,
           const dyn_humanoid::VecW& qd_max, double v_max_cart,
           const HumanoidZmpConstraint* zmp, ToppProfile& out) const;

 private:
  Config config_;

  // 力矩 ∩ ZMP 的联合 s̈ 边界；区间为空返回 PPH_ERR_TOPP_INFEASIBLE。
  static int ComputeCombinedBounds_(
      const dyn_humanoid::HumanoidDynFacade& facade,
      const HumanoidZmpConstraint* zmp,
      const WholeBodyPathSamples& path, int i, double sd,
      const dyn_humanoid::VecW& tau_max, const dyn_humanoid::VecW& tau_min,
      double& lo_out, double& hi_out);

  void BuildTrapezoidalFallback_(const WholeBodyPathSamples& path,
                                 ToppProfile& out) const;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TOPP_PROFILER_HPP_
