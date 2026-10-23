#ifndef TIME_SCALING_FEASIBILITY_HPP
#define TIME_SCALING_FEASIBILITY_HPP
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "time_optimal_profiler.hpp"
#include "dynamic_planner.hpp"

namespace action {

class TimeScalingFeasibility {
 public:
  // Given a profile originally of duration T_old and a target duration T_new,
  // compute the largest uniform scaling alpha = T_old / T_new such that the
  // scaled profile is torque-feasible. Returns false if the profile itself is
  // infeasible or the dynamics backend fails.
  bool ComputeMaxScale(const TimeOptimalProfiler::Profile& profile,
                       double target_duration_s,
                       pathplanning::DynamicPlanner& dyn,
                       double& alpha_out) const;
};

}  // namespace action

#endif  // TIME_SCALING_FEASIBILITY_HPP
#endif  // ROBOT_TYPE == DUAL_ARM
