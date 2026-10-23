#ifndef FUNC_PPWODYN_INCLUDE_SINGLE_SOC_KINEMATIC_PIPELINE_HPP_
#define FUNC_PPWODYN_INCLUDE_SINGLE_SOC_KINEMATIC_PIPELINE_HPP_

#include "path_action_types.hpp"
#include "action.hpp"
#include "composite_robot_model.hpp"
#include "kinematics_resolver.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "nie_group.hpp"
#endif
#include "ppwodyn_config.hpp"
#include "kinematic_topp_profiler.hpp"
#include "sync_trajectory_emitter.hpp"
#include "corner_blender.hpp"
#include "confbase/dynamics_config_adapter.hpp"
#include <deque>
#include <vector>

namespace ppwodyn {

// 单 SOC 规划管线（无动力学版）。
// 拷贝改造自 action::SingleSocPipeline（func/pathplanning）：
//   - 删 Step7（关节动力学富化）、GenrArmDyn、DynamicPlanner、UnifiedDynamicsModel
//   - 时间参数化由 KinematicToppProfiler（运动学 TOPP）完成，梯形兜底保留
//   - 拐角几何仍由 action::CornerBlender 完成（不拷贝）
class SingleSocKinematicPipeline {
 public:
  struct Result {
    bool ok{false};
    action::PlanError error{action::PlanError::kOk};
    char message[256]{};
    bool kinematics_degraded{false};  // 宽松模式下发生过 IK 失败降级
  };

  SingleSocKinematicPipeline();
  ~SingleSocKinematicPipeline();

  bool Init(const robot_config::DynamicsConfigAdapter& adapter,
            const action::CompositeRobotModel::SocDesc& soc_desc,
            const PpwodynConfig& config,
            CTargetGroup* shared_nie_group);

  int GetSocIndex() const { return soc_desc_.soc_index; }

  Result RunGeometricPhase(const std::vector<CAction>& actions);

  action::PathActionSequence& GetGeometricResult() { return result_; }
  const action::PathActionSequence& GetGeometricResult() const { return result_; }

  Result RunIndependentTimeOptimal();

  Result RunValidation();

  const action::PathActionSequence& GetResult() const { return result_; }

  const KinematicToppProfiler::Profile& GetProfile() const;

  void ApplySynchronizedTiming(const SyncTrajectoryEmitter::Result& timing);

  // fallback 速度剖面(梯形)。2026-08-26 起为 public static: 与管线内部
  // fallback 同一实现, 暴露给单测做域一致性白盒验证(项4)。
  // 注意: 写入的 v_enter/v_exit/v_cruise 为归一化 L 域 dL/dt。
  static bool ApplyFallbackVelocityProfile(action::PathActionSequence& result,
                                           const PpwodynConfig& config);

 private:
  PpwodynConfig config_;
  action::CompositeRobotModel::SocDesc soc_desc_;

  std::unique_ptr<action::KinematicsResolver> kinematics_resolver_;

  std::vector<CAction> actions_;
  std::vector<action::SegmentSpeedInfo> seg_speeds_;
  action::PathActionSequence result_;
  action::CornerBlender corner_blender_;
  std::deque<TargetData> blended_target_storage_;  // CornerBlender 输出存储
  // 2026-08-31 改 deque：多 line run 分段混合时 emplace_back 不失效已有
  // ActionData 指针（vector reserve 重分配会打断先行 run 的 Attach）。
  std::deque<ActionData> blended_action_data_;

  KinematicToppProfiler::Profile profile_;
  bool has_profile_{false};
  SyncTrajectoryEmitter::Result sync_timing_;
  bool has_sync_timing_{false};
  bool kinematics_degraded_{false};

  Result Step0_ResolveKinematics();
  Result Step2_ComputeSegmentMaxSpeeds();
  Result Step3_BuildPathActionSegments();
  Result Step4_CheckJointVelocityLimits();
  Result Step6_Validate();

  // 2026-08-31 joint 支持：多 run 时间参数化（joint 段 fine point 硬隔离）。
  Result RunMultiRunTimeOptimal(const std::vector<size_t>& run_starts);

  bool BuildBlendedActions(const std::vector<CAction>& actions,
                           std::vector<CAction>& out_actions);

  // 连续 LINE run [begin, end) 送 CornerBlender 细分，结果追加到 out_actions。
  bool BlendLineRun(const std::vector<CAction>& actions, size_t begin, size_t end,
                    std::vector<CAction>& out_actions);
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_SINGLE_SOC_KINEMATIC_PIPELINE_HPP_
