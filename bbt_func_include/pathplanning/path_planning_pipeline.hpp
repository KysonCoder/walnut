#ifndef PATH_PLANNING_PIPELINE_HPP
#define PATH_PLANNING_PIPELINE_HPP

#include "path_action_types.hpp"
#include "confbase/robot.hpp"
#include "velocity_profiler.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "dynamic_planner.hpp"
#include "joint_dynamics_enricher.hpp"
#endif
#include "planned_motion.hpp"
#include "kinematics_resolver.hpp"
#include "dynamics_config_adapter.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "nie_group.hpp"
#endif
#include "composite_robot_model.hpp"
#include "unified_dynamics_model.hpp"
#include "single_soc_pipeline.hpp"
#include "path_planning_config.hpp"
#include "multi_soc_coordinator.hpp"
#include "sync_trajectory_emitter.hpp"
#include "path_planning_limits_adapter.hpp"
#include <vector>
#include <memory>
#include <string>

namespace action {

class PathPlanningPipeline {
 public:
  using Config = PathPlanningConfig;
  using StepResult = PathPlanningStepResult;

  PathPlanningPipeline();
  explicit PathPlanningPipeline(const Config& config);
  ~PathPlanningPipeline();

  bool Init(const robot_config::DynamicsConfigAdapter& adapter,
            const std::string& base_link,
            const std::string& body_link);

  bool IsInitialized() const { return initialized_; }

  StepResult RunAll(const std::vector<const CTargetGroup*>& composite_waypoints);

  const PathActionGroup& GetResult() const { return result_group_; }
  const std::vector<PlanResult>& GetErrors() const { return errors_; }
  const std::vector<PlanResult>& GetWarnings() const { return warnings_; }

  bool WasPhase2Applied() const { return was_phase2_applied_; }
#if ROBOT_TYPE == DUAL_ARM
  const MultiSocCoordinator::Result* GetLastCoordinatorResult() const {
    return last_coord_result_.feasible ? &last_coord_result_ : nullptr;
  }
#endif

 private:
  bool BuildSocWaypoints(const std::vector<const CTargetGroup*>& composite_waypoints,
                         int soc_index,
                         std::vector<TargetData>& out_data,
                         std::vector<CTarget>& out_waypoints) const;
  bool BuildActionsFromWaypoints(const std::vector<CTarget>& waypoints,
                                 std::vector<ActionData>& out_data,
                                 std::vector<CAction>& out_actions) const;

  Config config_;
  robot_config::DynamicsConfigAdapter adapter_;
  pathplanning::PathPlanningLimitsAdapter limits_adapter_;
  std::string base_link_;
  std::string body_link_;

  std::unique_ptr<CompositeRobotModel> robot_model_;
#if ROBOT_TYPE == DUAL_ARM
  std::unique_ptr<UnifiedDynamicsModel> unified_dyn_;
#endif
  std::vector<std::unique_ptr<SingleSocPipeline>> soc_pipelines_;

#if ROBOT_TYPE == DUAL_ARM
  std::unique_ptr<MultiSocCoordinator> multi_soc_coordinator_;
  std::unique_ptr<SyncTrajectoryEmitter> sync_emitter_;
#endif

#if ROBOT_TYPE == DUAL_ARM
  std::unique_ptr<CNieGroup> nie_group_;
  std::unique_ptr<TargetGroupSharedData> nie_data_;
#endif

  // NIE 共享组访问：dualarm 返回 nie_group_，humanoid 恒 nullptr。
  CTargetGroup* NieGroupOrNull_() {
#if ROBOT_TYPE == DUAL_ARM
    return nie_group_ ? static_cast<CTargetGroup*>(nie_group_.get()) : nullptr;
#else
    return nullptr;
#endif
  }


  // Persistent storage for targets/actions built during RunAll; the view
  // classes (CTarget/CAction) hold pointers to this data.
  std::vector<std::vector<TargetData>> soc_waypoint_data_;
  std::vector<std::vector<ActionData>> soc_action_data_;

  PathActionGroup result_group_;
  std::vector<PlanResult> errors_;
  std::vector<PlanResult> warnings_;
  bool was_phase2_applied_ = false;
#if ROBOT_TYPE == DUAL_ARM
  MultiSocCoordinator::Result last_coord_result_{};
#endif
  bool initialized_ = false;
};

}  // namespace action

#endif  // PATH_PLANNING_PIPELINE_HPP
