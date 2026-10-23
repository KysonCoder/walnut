// func/pathplanning_humanoid/include/pphumanoid_types.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_PPHUMANOID_TYPES_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_PPHUMANOID_TYPES_HPP_

#include <array>
#include <string>
#include <vector>

#include "dyn_humanoid_types.hpp"   // dyn_humanoid::VecW/Vec2/kWholeBodyDof/kMaxSupportPoints
#include "path_action_types.hpp"    // action::PathActionGroup

namespace pphumanoid {

// ---- 错误码（-3500~-3599，避开 dyn_humanoid 的 -30xx/-31xx/-33xx）----
constexpr int PPH_OK                        = 0;
constexpr int PPH_ERR_NOT_INITED            = -3501;
constexpr int PPH_ERR_IK_FAILED             = -3502;
constexpr int PPH_ERR_TOPP_INFEASIBLE       = -3503;
constexpr int PPH_ERR_NO_SUPPORT_POLYGON    = -3504;
constexpr int PPH_ERR_TORQUE_LIMITS_MISSING = -3505;
constexpr int PPH_ERR_LIMIT_VIOLATION       = -3506;
constexpr int PPH_ERR_BAD_INPUT             = -3507;

const char* ErrorCodeToString(int code);

// ---- 支撑多边形配置（启用 ZMP 约束时必传；无默认值是显式设计）----
struct SupportPolygonConfig {
  std::array<dyn_humanoid::Vec2, dyn_humanoid::kMaxSupportPoints> points_base{};
  int num_points = 0;      // 3..8；0 = 未设置
  double ground_z = 0.0;
};

// ---- 规划配置 ----
struct HumanoidPlanConfig {
  double sample_period_s = 0.001;       // 轨迹输出周期
  int    topp_samples = 500;            // TOPP 路径参数采样数
  double corner_radius_m = 0.01;        // 拐角混合半径
  double corner_sample_step_m = 0.001;  // 拐角采样步长
  double v_max_cartesian = 0.25;        // 笛卡尔速度上限 [m/s]
  double a_max_cartesian = 1.0;         // 回退梯形用加速度上限 [m/s^2]
  double fallback_v_max_cart = 0.1;     // TOPP 不可行回退速度 [m/s]

  bool   zmp_constraint_enabled = true;
  double zmp_safety_margin = 0.03;      // ZMP 最小内缩裕度 [m]
  SupportPolygonConfig support_polygon; // enabled 时 num_points 必须 >= 3

  bool has_tau_override = false;        // 力矩上限覆盖（drivechain 缺失时用）
  dyn_humanoid::VecW tau_max_override{};
  dyn_humanoid::VecW tau_min_override{};

  bool strict_kinematics = true;        // IK 失败即报错；false = 跳过该点+warning
  bool singularity_scan_enabled = false; // 规划后可操作性扫描（可选）
};

// ---- 全身路径采样表（TOPP 输入；s ∈ [0,1] 全身共享）----
struct WholeBodyPathSamples {
  int num_samples = 0;                  // 采样点数（含两端）
  double ds = 0.0;                      // 1.0/(num_samples-1)
  std::vector<dyn_humanoid::VecW> q;        // q(s)
  std::vector<dyn_humanoid::VecW> dq_ds;    // q'(s) 数值微分
  std::vector<dyn_humanoid::VecW> d2q_ds2;  // q''(s) 数值微分
  double ref_cart_length_m = 0.0;       // 各臂最大弧长（笛卡尔限速折算用）
};

// ---- TOPP 输出 ----
struct ToppProfile {
  std::vector<double> s;     // 路径参数采样
  std::vector<double> sd;    // ṡ = ds/dt [1/s]
  std::vector<double> t;     // 累积时间 [s]
  std::vector<double> sdd;   // s̈ [1/s^2]
  bool feasible = false;
  bool used_fallback = false;  // true = 梯形回退
};

// ---- ZMP 规划后报告 ----
struct ZmpStabilityReport {
  std::vector<double> margins;   // 每采样点 ZMP 裕度 [m]（内正外负）
  double min_margin = 0.0;
  int min_index = -1;
  int singular_count = 0;        // ERR_ZMP_SINGULAR 出现次数
};

// ---- 管线问题记录 ----
struct PlanIssue {
  int code = 0;
  std::string message;
};

// ---- 管线输出 ----
struct HumanoidPlanResult {
  action::PathActionGroup motion;   // kinplanning 数据模型，下游零适配
  ZmpStabilityReport stability;
  std::vector<double> manipulability_min_per_arm;  // singularity_scan_enabled 时填充
  std::vector<PlanIssue> errors;
  std::vector<PlanIssue> warnings;
  bool used_fallback = false;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_PPHUMANOID_TYPES_HPP_
