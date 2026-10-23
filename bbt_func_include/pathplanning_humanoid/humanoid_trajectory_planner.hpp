// func/pathplanning_humanoid/include/humanoid_trajectory_planner.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TRAJECTORY_PLANNER_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TRAJECTORY_PLANNER_HPP_

#include "humanoid_path_pipeline.hpp"

namespace pphumanoid {

// 便捷单入口（镜像老库 TrajectoryPlanner 角色）：持有一个 pipeline，
// 每次 Plan 先 Init（幂等）再 RunAll。config 变化自然生效。
class HumanoidTrajectoryPlanner {
 public:
  HumanoidTrajectoryPlanner() = default;
  bool IsInited() const { return pipeline_.IsInited(); }

  int Plan(robot_config::DynamicsConfigAdapter& adapter,
           const std::vector<const CTargetGroup*>& groups,
           const dyn_humanoid::VecW& q_current,
           const HumanoidPlanConfig& config, HumanoidPlanResult& out);

 private:
  HumanoidPathPipeline pipeline_;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_TRAJECTORY_PLANNER_HPP_
