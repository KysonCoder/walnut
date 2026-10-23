#ifndef FUNC_PPWODYN_INCLUDE_KINEMATIC_TOPP_PROFILER_HPP_
#define FUNC_PPWODYN_INCLUDE_KINEMATIC_TOPP_PROFILER_HPP_

#include "path_action_types.hpp"   // kinplanning: action::PathActionSequence
#include "ppwodyn_config.hpp"
#include "math/se3_twist_ops.hpp"
#include <cstdint>
#include <vector>

namespace ppwodyn {

// 运动学时间最优路径参数化（无动力学）。
// 拷贝改造自 action::TimeOptimalProfiler（func/pathplanning）：
//   - 速度上限: ṡ_max = min( min_j v_joint_max/|dq_j/dL|, v_max/|dp/dL| )
//   - 加速度包络: |d²q/dL²·ṡ² + dq/dL·s̈| ≤ a_joint_max（逐关节线性不等式求交）
//                 笛卡尔项同理（保守标量形式）
//   - 退化 → 段级梯形兜底（GenerateFallbackProfile）
//
// 量纲约定（与原 pathplanning 实现不同，本库内部自洽）：
//   L ∈ [0,1] 为整条序列的归一化路径参数；所有 dq/dL、dp/dL 均折算到该域。
//   段数据 end_dyn.dq_dL 是段级关节差分（rad/段），读入后按
//   (Δq/seg_len)·total_length 折算；v_max_cartesian(m/s) 钳制按
//   ṡ_L ≤ v_max/total_length 折算。
class KinematicToppProfiler {
 public:
  struct Profile {
    std::vector<double> L;
    std::vector<double> v;
    std::vector<double> t;
    std::vector<double> a;
    std::vector<action::SegmentJointArray> q;         // 每采样点关节角(7 宽, 2026-08-26 项3)
    std::vector<action::SegmentJointArray> dq_dL;     // dq/dL（归一化 L 域，供时间缩放复用）
    std::vector<action::SegmentJointArray> d2q_dL2;   // d²q/dL²
    std::vector<double> dp_dL_norm;         // |dp/dL|（笛卡尔）
    std::vector<double> d2p_dL2_norm;       // |d²p/dL²|（保守标量）
    std::vector<uint8_t> is_joint;  // 2026-09-02 R4: 每样本是否 joint 段
                                    // （1=joint，时间缩放时跳过笛卡尔约束项）
    bool feasible{false};
    bool used_fallback{false};
  };

  struct Config {
    int num_samples;       // 路径采样分辨率
    double delta_L;        // 差分步长
    Config() : num_samples(500), delta_L(1e-4) {}
  };

  explicit KinematicToppProfiler(const Config& config = Config());

  // 复合管线流程入口（PathActionSequence）。limits 来自 PpwodynConfig
  // （一般经 PpwodynLimitsAdapter 填充）。
  bool Plan(const action::PathActionSequence& seq,
            const KinematicLimits& limits,
            Profile& out);

  void SetFallbackLimits(double v_max, double a_max) {
    if (v_max > 0.0 && a_max > 0.0) {
      fallback_v_max_ = v_max;
      fallback_a_max_ = a_max;
    }
  }

  // IK 恢复三件套（语义同 action::TimeOptimalProfiler）：
  // SetKinematicContext 提供逐点 IK 所需的运动学上下文；
  // SetIkSeed 固定首次求解的分支，且在每次 Plan 开始时重放。
  void SetKinematicContext(const MAN_ROBOT& robot_param, const FRAME& tool_frame) {
    robot_param_ = robot_param;
    tool_frame_ = tool_frame;
    has_kinematic_context_ = true;
  }
  void SetIkRecoveryEnabled(bool enabled) { ik_recovery_enabled_ = enabled; }
  void SetIkSeed(const SOC_JOINTS& joints) {
    external_ik_seed_ = joints;
    external_ik_config_ = TARGET_CONFIG{};
    has_external_ik_seed_ = true;
  }
  void SetIkSeed(const SOC_JOINTS& joints, const TARGET_CONFIG& config) {
    external_ik_seed_ = joints;
    external_ik_config_ = config;
    has_external_ik_seed_ = true;
  }

 private:
  Config config_;
  double fallback_v_max_ = 0.5;
  double fallback_a_max_ = 2.0;

  MAN_ROBOT robot_param_{};
  FRAME tool_frame_{};
  bool has_kinematic_context_ = false;
  bool ik_recovery_enabled_ = true;
  SOC_JOINTS last_ik_seed_{};
  TARGET_CONFIG last_ik_config_{};
  bool has_ik_seed_ = false;
  SOC_JOINTS external_ik_seed_{};
  TARGET_CONFIG external_ik_config_{};
  bool has_external_ik_seed_ = false;

  // 重启 IK 链：新采样轮次从外部种子（若有）重新开始。
  void ResetIkChain_() {
    has_ik_seed_ = false;
    if (has_external_ik_seed_) {
      last_ik_seed_ = external_ik_seed_;
      last_ik_config_ = external_ik_config_;
      has_ik_seed_ = true;
    }
  }

  // IK 恢复：由纯位姿目标解关节角，逐点串联保持分支连续。
  bool RecoverJoints_(::CTarget& target);

  void ComputeParametricFallbackLimits(double physical_length,
                                       double parametric_length,
                                       double* out_param_v_max,
                                       double* out_param_a_max) const;

  // —— 运动学约束（替换 DynamicPlanner 的两个调用点）——

  // 速度上限：ṡ_max = min( min_j v_joint_max/|dq_j/dL|, v_max/|dp/dL|,
  // 向心项 min_j √(a_joint_max/|d2q_j/dL2|), √(a_max/|d2p/dL2|) )。
  // 2026-10-12 PS1 连带修：向心速度钳是标准 TOPP 速度边界——此前缺失，
  // 前向积分在尖锐拐角/高曲率样本超速后由加速度包络校验直接判 false
  // （假不可行→fallback），速度恢复物理量级后暴露。
  double ComputeKinematicVelocityLimit_(const action::SegmentJointArray& dq_dL,
                                        const action::SegmentJointArray& d2q_dL2,
                                        double dp_dL_norm, double d2p_dL2_norm,
                                        const KinematicLimits& limits) const;

  // 加速度包络 [L_out, U_out]：逐关节解 |d2q·v² + dq·u| ≤ a_joint_max，
  // 笛卡尔用保守标量 |d2p|·v² + |dp|·|u| ≤ a_max；静止段用 ±fallback_a_max_ 封顶。
  bool ComputeKinematicAccBounds_(const action::SegmentJointArray& dq_dL,
                                  const action::SegmentJointArray& d2q_dL2,
                                  double dp_dL_norm, double d2p_dL2_norm,
                                  double v, const KinematicLimits& limits,
                                  double& L_out, double& U_out) const;

  void ForwardPass_(const std::vector<double>& U, double delta_L,
                    std::vector<double>& v_fwd);
  void BackwardPass_(const std::vector<double>& L_bound, double delta_L,
                     std::vector<double>& v_bwd);
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_KINEMATIC_TOPP_PROFILER_HPP_
