#ifndef FUNC_PPWODYN_INCLUDE_PPWODYN_CONFIG_HPP_
#define FUNC_PPWODYN_INCLUDE_PPWODYN_CONFIG_HPP_

namespace ppwodyn {

// 纯运动学限幅（替代 action::VelocityProfiler::VelocityLimits，默认值逐项对齐）。
// POD：共享内存兼容（CLAUDE.md §3）。
struct KinematicLimits {
  double v_max{0.25};        // 笛卡尔线速度上限 [m/s]
  double a_max{1.0};         // 笛卡尔加速度上限 [m/s²]
  double j_max{10.0};        // 保留字段，当前算法不使用 jerk 限幅
  double v_joint_max{1.05};  // 关节速度上限 [rad/s]
  double a_joint_max{1.05};  // 关节加速度上限 [rad/s²]
};

// 2026-10-12 PS1：NSDMI 默认值即「未配置/未 override」标记值。
// 0.25 为手动模式限速（规划侧已揭开）；override 传入标记值/<=0/NaN
// 一律视为未设置该字段（值初始化 KinematicLimits{} 即可安全地只填其一）。
constexpr double kKinematicLimitsManualVMax = 0.25;
constexpr double kKinematicLimitsDefaultVJointMax = 1.05;

constexpr int kMaxPpwodynMessageLength = 256;

struct PpwodynConfig {
  double sample_period_s{0.024};
  double corner_radius_m{0.01};        // 保留对齐（见 pathplanning 同名字段注释）
  double corner_sample_step_m{0.001};  // 保留对齐
  KinematicLimits vel_limits;

  // Corner blending（action::CornerBlender, parabolic+sine 预采样）
  double corner_blend_radius_m{0.0};        // 0 = 禁用（纯直线）
  double corner_blend_sample_step_m{0.001}; // 曲线采样步长 [m]

  // Step0 kinematics resolution
  bool strict_kinematics{true};

  // Multi-SOC coordinator
  bool enable_phase2_multi_soc{false};
  int num_output_samples{100};
  int coordinator_max_iterations{50};
  double coordinator_time_tolerance_s{1e-4};
  double coordinator_max_duration_s{60.0};
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_PPWODYN_CONFIG_HPP_
