#ifndef FUNC_PPWODYN_INCLUDE_MULTI_SOC_KINEMATIC_COORDINATOR_HPP_
#define FUNC_PPWODYN_INCLUDE_MULTI_SOC_KINEMATIC_COORDINATOR_HPP_

#include "kinematic_topp_profiler.hpp"
#include "ppwodyn_config.hpp"
#include <vector>

namespace ppwodyn {

// 多 SOC 时长同步协调器（运动学版）。
// 拷贝自 action::MultiSocCoordinator：二分搜索公共时长，可行性检查由
// KinematicTimeScaling（运动学限幅）替代原 TimeScalingFeasibility（动力学力矩）。
class MultiSocKinematicCoordinator {
 public:
  struct SocInput {
    KinematicToppProfiler::Profile profile;
  };

  struct Result {
    bool feasible{false};
    double common_duration_s{0.0};
    std::vector<double> per_soc_scale;  // T_old_i / common_duration_s
    int iterations{0};
    // 2026-09-01: 输入结构性不支持（如多 run/混合序列拼接 profile 缺 a 列），
    // 区别于"时序不可行"。编排器应对此硬报错而非降级为独立时序。
    bool unsupported_input{false};
    char message[256]{};
  };

  MultiSocKinematicCoordinator() = default;

  bool Optimize(const std::vector<SocInput>& inputs,
                const PpwodynConfig& config,
                Result& out);
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_MULTI_SOC_KINEMATIC_COORDINATOR_HPP_
