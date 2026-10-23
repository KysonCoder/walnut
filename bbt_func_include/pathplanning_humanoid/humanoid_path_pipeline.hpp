// func/pathplanning_humanoid/include/humanoid_path_pipeline.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_PIPELINE_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_PIPELINE_HPP_

#include "composite_robot_model.hpp"
#include "humanoid_dyn_facade.hpp"
#include "humanoid_limits_adapter.hpp"
#include "humanoid_path_generator.hpp"
#include "humanoid_topp_profiler.hpp"
#include "humanoid_zmp_constraint.hpp"
#include "kinematics_resolver.hpp"
#include "pphumanoid_types.hpp"
#include "whole_body_path_assembler.hpp"

namespace robot_config {
class DynamicsConfigAdapter;
}

namespace pphumanoid {

// 人形全身动力学路径规划管线（镜像老库 PathPlanningPipeline 的角色）。
// 输入：复合路点序列（每元素一个 CTargetGroup，持所有 SOC）。
// 输出：PathActionGroup（下游零适配）+ ZMP 报告 + 问题列表。
class HumanoidPathPipeline {
 public:
  HumanoidPathPipeline() = default;

  // adapter: 已 attach 到 HumanoidConfigStore 的 DynamicsConfigAdapter。
  // config.zmp_constraint_enabled=true 时 support_polygon.num_points>=3 必填。
  int Init(robot_config::DynamicsConfigAdapter& adapter,
           const HumanoidPlanConfig& config);
  bool IsInited() const { return inited_; }

  // q_current: 全身当前位形（腰/头/未使能关节的保持值 + IK 起始种子）。
  int RunAll(const std::vector<const CTargetGroup*>& groups,
             const dyn_humanoid::VecW& q_current, HumanoidPlanResult& out);

  // 组件只读访问（测试/E2E 导出用）
  const dyn_humanoid::HumanoidDynFacade& Facade() const { return facade_; }

 private:
  HumanoidPlanConfig config_;
  dyn_humanoid::HumanoidDynFacade facade_;
  action::KinematicsResolver resolver_;
  action::CompositeRobotModel crm_;
  HumanoidLimitsAdapter limits_;
  HumanoidZmpConstraint zmp_;
  bool zmp_ready_ = false;
  bool inited_ = false;

  // Step5: 把 TOPP 结果填进 PathActionSequence（单臂）
  int FillArmSequence_(
      int arm_soc, dyn_humanoid::Chain chain,
      const std::vector<TargetData>& waypoint_data,
      const std::vector<double>& waypoint_arcs, double arm_length,
      const WholeBodyPathSamples& samples, const ToppProfile& profile,
      action::PathActionSequence& seq_out);
  // Step6: 规划后 ZMP 全报告
  void BuildPostReport_(const WholeBodyPathSamples& samples,
                        const ToppProfile& profile, HumanoidPlanResult& out);
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_PIPELINE_HPP_
