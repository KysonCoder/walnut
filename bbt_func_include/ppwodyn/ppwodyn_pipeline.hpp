#ifndef FUNC_PPWODYN_INCLUDE_PPWODYN_PIPELINE_HPP_
#define FUNC_PPWODYN_INCLUDE_PPWODYN_PIPELINE_HPP_

#include "path_action_types.hpp"
#include "composite_robot_model.hpp"
#include "kinematics_resolver.hpp"
#include "confbase/dualarm_conf.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "nie_group.hpp"
#endif
#include "ppwodyn_config.hpp"
#include "ppwodyn_limits_adapter.hpp"
#include "single_soc_kinematic_pipeline.hpp"
#include "multi_soc_kinematic_coordinator.hpp"
#include "sync_trajectory_emitter.hpp"
#include <vector>
#include <memory>
#include <string>

namespace ppwodyn {

// 步骤结果（对齐 action::PathPlanningStepResult，定义在此以保持
// ppwodyn_config.hpp 纯 POD）。
struct PpwodynStepResult {
  bool ok{false};
  action::PlanError error{action::PlanError::kOk};
  char message[kMaxPpwodynMessageLength]{};
  int soc_index{-1};
};

// 复合机器人无动力学规划门面。
// 拷贝改造自 action::PathPlanningPipeline（func/pathplanning）：
//   - 删 UnifiedDynamicsModel / FillTorqueLimits13 / GetDynamicsBackend 链路
//   - 时间参数化由 SingleSocKinematicPipeline（KinematicToppProfiler）完成
//   - phase2 多 SOC 同步：MultiSocKinematicCoordinator + SyncTrajectoryEmitter
class PpwodynPipeline {
 public:
  using Config = PpwodynConfig;
  using StepResult = PpwodynStepResult;

  PpwodynPipeline();
  explicit PpwodynPipeline(const Config& config);
  ~PpwodynPipeline();

  // base_link/body_link 保留参数位（原版供 UnifiedDynamicsModel 使用，本库不用）。
  bool Init(const robot_config::DynamicsConfigAdapter& adapter,
            const std::string& base_link,
            const std::string& body_link);

  bool IsInitialized() const { return initialized_; }

  StepResult RunAll(const std::vector<const CTargetGroup*>& composite_waypoints);

  // 2026-10-12 PS1：逐组限幅 override。nullptr 或字段 <=0/NaN → 该字段用
  // 管线填充后默认；正值字段 → min(override, 填充后默认)（只收不放）。
  // 仅 v_max/v_joint_max 两字段生效，其余字段忽略。
  StepResult RunAll(const std::vector<const CTargetGroup*>& composite_waypoints,
                    const KinematicLimits* limits_override);

  const action::PathActionGroup& GetResult() const { return result_group_; }
  const std::vector<action::PlanResult>& GetErrors() const { return errors_; }
  const std::vector<action::PlanResult>& GetWarnings() const { return warnings_; }

  bool WasPhase2Applied() const { return was_phase2_applied_; }
  const MultiSocKinematicCoordinator::Result* GetLastCoordinatorResult() const {
    return last_coord_result_.feasible ? &last_coord_result_ : nullptr;
  }

 private:
  bool BuildSocWaypoints(const std::vector<const CTargetGroup*>& composite_waypoints,
                         int soc_index,
                         std::vector<TargetData>& out_data,
                         std::vector<CTarget>& out_waypoints) const;
  // 2026-09-02 R1: 按终点 motion_type 分派；不支持的类型写 err_out 并返回 false
  bool BuildActionsFromWaypoints(const std::vector<CTarget>& waypoints,
                                 std::vector<ActionData>& out_data,
                                 std::vector<CAction>& out_actions,
                                 StepResult* err_out) const;
  // 压缩 NULL 端点：保留 [0]，剔除 i>=1 中 motion_type==ACTION_NULL 的端点。
  // 输入为 BuildSocWaypoints 抽取的原始数组；压缩后返回有效端点数。
  static size_t CompactNullWaypoints(std::vector<TargetData>& data);

  Config config_;
  robot_config::DynamicsConfigAdapter adapter_;
  PpwodynLimitsAdapter limits_adapter_;

  std::unique_ptr<action::CompositeRobotModel> robot_model_;
  std::vector<std::unique_ptr<SingleSocKinematicPipeline>> soc_pipelines_;

  std::unique_ptr<MultiSocKinematicCoordinator> multi_soc_coordinator_;
  std::unique_ptr<SyncTrajectoryEmitter> sync_emitter_;

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

  action::PathActionGroup result_group_;
  std::vector<action::PlanResult> errors_;
  std::vector<action::PlanResult> warnings_;
  bool was_phase2_applied_ = false;
  MultiSocKinematicCoordinator::Result last_coord_result_{};
  bool initialized_ = false;
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_PPWODYN_PIPELINE_HPP_
