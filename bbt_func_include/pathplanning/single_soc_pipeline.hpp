#ifndef SINGLE_SOC_PIPELINE_HPP
#define SINGLE_SOC_PIPELINE_HPP

#include "path_action_types.hpp"
#include "confbase/robot.hpp"
#include "action.hpp"
#include "composite_robot_model.hpp"
#include "kinematics_resolver.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "GenrArmDyn.hpp"
#endif
#include "dynamics_config_adapter.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "nie_group.hpp"
#endif
#include "path_planning_config.hpp"
#include "unified_dynamics_model.hpp"
#include "time_optimal_profiler.hpp"
#include "sync_trajectory_emitter.hpp"
#include "corner_blender.hpp"
#include <deque>
#include <memory>
#include <vector>

namespace action {

class SingleSocPipeline {
 public:
  struct Result {
    bool ok{false};
    PlanError error{PlanError::kOk};
    char message[256]{};
    bool kinematics_degraded{false};  // 宽松模式下发生过 IK 失败降级
  };

  SingleSocPipeline();
  ~SingleSocPipeline();

  bool Init(const robot_config::DynamicsConfigAdapter& adapter,
            const CompositeRobotModel::SocDesc& soc_desc,
            const PathPlanningConfig& config,
            CTargetGroup* shared_nie_group);

  int GetSocIndex() const { return soc_desc_.soc_index; }

  Result RunGeometricPhase(const std::vector<CAction>& actions);

  PathActionSequence& GetGeometricResult() { return result_; }
  const PathActionSequence& GetGeometricResult() const { return result_; }

#if ROBOT_TYPE == DUAL_ARM
  bool ApplyUnifiedTiming(const UnifiedDynamicsResult& timing);
#endif

  Result RunIndependentTimeOptimal();

  // DUAL_ARM: Step6 validation + Step7 dynamics enrichment.
  // HUMANOID: Step6 validation only (no dynamics in this flavor).
  Result RunValidationAndDynamics();

  const PathActionSequence& GetResult() const { return result_; }

#if ROBOT_TYPE == DUAL_ARM
  const TimeOptimalProfiler::Profile& GetProfile() const;

  // Returns the per-SOC dynamics backend owned by this pipeline. The pointer
  // is valid only for the lifetime of this pipeline and must not be deleted.
  pathplanning::DynamicPlanner* GetDynamicsBackend() const;
#endif

#if ROBOT_TYPE == DUAL_ARM
  void ApplySynchronizedTiming(const SyncTrajectoryEmitter::Result& timing);
#endif

 private:
  PathPlanningConfig config_;
  CompositeRobotModel::SocDesc soc_desc_;

#if ROBOT_TYPE == DUAL_ARM
  std::unique_ptr<interpolation::GenrArmDyn> genr_arm_dyn_;
#endif
  std::unique_ptr<KinematicsResolver> kinematics_resolver_;

  std::vector<CAction> actions_;
  std::vector<SegmentSpeedInfo> seg_speeds_;
  PathActionSequence result_;
  CornerBlender corner_blender_;
  std::deque<TargetData> blended_target_storage_;  // CornerBlender 输出存储
  std::vector<ActionData> blended_action_data_;

#if ROBOT_TYPE == DUAL_ARM
  std::unique_ptr<pathplanning::DynamicPlanner> dynamic_planner_;
  TimeOptimalProfiler::Profile profile_;
  bool has_profile_{false};
#endif
#if ROBOT_TYPE == DUAL_ARM
  SyncTrajectoryEmitter::Result sync_timing_;
  bool has_sync_timing_{false};
#endif
  bool kinematics_degraded_{false};

  Result Step0_ResolveKinematics();
  Result Step2_ComputeSegmentMaxSpeeds();
  Result Step3_BuildPathActionSegments();
  Result Step4_CheckJointVelocityLimits();
  Result Step6_Validate();
#if ROBOT_TYPE == DUAL_ARM
  Result Step7_ComputeJointDynamics();
#endif

  bool BuildBlendedActions(const std::vector<CAction>& actions,
                           std::vector<CAction>& out_actions);

  static bool ApplyFallbackVelocityProfile(PathActionSequence& result,
                                            const PathPlanningConfig& config);
};

}  // namespace action

#endif  // SINGLE_SOC_PIPELINE_HPP
