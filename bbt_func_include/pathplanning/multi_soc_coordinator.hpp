#ifndef MULTI_SOC_COORDINATOR_HPP
#define MULTI_SOC_COORDINATOR_HPP
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "time_optimal_profiler.hpp"
#include "dynamic_planner.hpp"
#include "path_planning_config.hpp"
#include <vector>

namespace action {

class MultiSocCoordinator {
 public:
  struct SocInput {
    TimeOptimalProfiler::Profile profile;
    pathplanning::DynamicPlanner* dynamics{nullptr};
  };

  struct Result {
    bool feasible{false};
    double common_duration_s{0.0};
    std::vector<double> per_soc_scale;  // T_old_i / common_duration_s
    int iterations{0};
    char message[256]{};
  };

  MultiSocCoordinator() = default;

  bool Optimize(const std::vector<SocInput>& inputs,
                const PathPlanningConfig& config,
                Result& out);
};

}  // namespace action

#endif  // MULTI_SOC_COORDINATOR_HPP
#endif  // ROBOT_TYPE == DUAL_ARM
