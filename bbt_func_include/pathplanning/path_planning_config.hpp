#ifndef PATH_PLANNING_CONFIG_HPP
#define PATH_PLANNING_CONFIG_HPP

#include "velocity_profiler.hpp"
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "dynamic_planner.hpp"
#include "joint_dynamics_enricher.hpp"
#include "dual_arm_dyn.hpp"
#endif
#include "trajectory_types.hpp"

namespace action {

constexpr int kMaxPathPlanningMessageLength = 256;

#if ROBOT_TYPE == DUAL_ARM
struct UnifiedDynamicsConfig {
  int num_samples{100};                    // phase-plane sampling resolution
  int velocity_limit_iters{10};            // bisection iterations for max ds/dt
  double max_ds_dt{10.0};                  // upper search bound for ds/dt [1/s]
  double infeasible_slowdown_factor{1.25}; // uniform fallback slowdown multiplier
  int max_fallback_iterations{5};          // max fallback attempts
  interpolation::Vec13 tau_max{};          // 13-DOF torque limits [Nm]
};
#endif

struct PathPlanningConfig {
  double sample_period_s{0.024};
  // deprecated(2026-07-20): 旧 ActionBase corner 公式路径专用，公式代码已删除。
  // 字段保留仅为避免影响既有序列化/初始化代码；corner 过渡统一由
  // corner_blend_radius_m + corner_blend_sample_step_m 控制。
  double corner_radius_m{0.01};
  double corner_sample_step_m{0.001};
  VelocityProfiler::VelocityLimits vel_limits;
#if ROBOT_TYPE == DUAL_ARM
  pathplanning::DynamicPlanner::Config dyn_config;
#endif
  double jacobian_safety_factor{0.9};
#if ROBOT_TYPE == DUAL_ARM
  JointDynamicsEnricher::Config enricher_config;
  bool use_unified_dynamics{true};
  UnifiedDynamicsConfig unified_dynamics;
#endif

  // Corner blending (CornerBlender, parabolic+sine 预采样)
  double corner_blend_radius_m{0.0};        // 0 = disabled（纯直线，无拐角过渡）
  double corner_blend_sample_step_m{0.001}; // 曲线采样步长 [m]

  // Step0 kinematics resolution
  bool strict_kinematics{true};  // true: IK 失败即 kIKFailure；false: 宽松 + 降级标记

  // Multi-SOC coordinator
  bool enable_phase2_multi_soc{false};  // true to run coupled coordinator
  int num_output_samples{100};          // synchronized trajectory samples
  int coordinator_max_iterations{50};
  double coordinator_time_tolerance_s{1e-4};  // convergence tolerance [s]
  double coordinator_max_duration_s{60.0};    // upper search bound [s]
};

struct PathPlanningStepResult {
  bool ok{false};
  PlanError error{PlanError::kOk};
  char message[kMaxPathPlanningMessageLength]{};
  int soc_index{-1};
};

}  // namespace action

#endif  // PATH_PLANNING_CONFIG_HPP
