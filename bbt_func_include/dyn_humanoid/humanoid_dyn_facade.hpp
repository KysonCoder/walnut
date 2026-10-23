// func/dyn_humanoid/include/humanoid_dyn_facade.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_FACADE_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_FACADE_HPP_

#include "dex_hand_dyn.hpp"
#include "dyn_humanoid_types.hpp"
#include "humanoid_centroidal.hpp"
#include "humanoid_wbd.hpp"

namespace robot_config {
class DynamicsConfigAdapter;
}

namespace dyn_humanoid {

// ComputeFeedforward 的输出包。
struct FeedforwardResult {
  VecW tau_whole_body{};
  VecH tau_left_hand{};
  VecH tau_right_hand{};
  int wbd_rc = 0;
  int hand_rc[2] = {0, 0};
};

// 双用途外观（spec §7）：持有 Wbd + Centroidal + 左右 DexHand，
// 只做组合不做算法。
//   控制前馈：ComputeFeedforward（1kHz 热路径，无堆分配/无异常/无虚函数）
//   规划：    AccelBounds / StabilityMargin（逐路径点调用）
//
// 阶段一为骨架：Init 返回 NOT_IMPLEMENTED；ComputeAccelBoundsCore
// （纯数学）为真实实现；组件直通访问器可用。
class HumanoidDynFacade {
 public:
  static constexpr int DYN_FACADE_OK                  = 0;
  static constexpr int DYN_FACADE_ERR_NOT_INITED      = -3301;
  static constexpr int DYN_FACADE_ERR_LIMITS_MISSING  = -3302;
  static constexpr int DYN_FACADE_ERR_INFEASIBLE      = -3303;
  static constexpr int DYN_FACADE_ERR_NOT_IMPLEMENTED = -3398;
  static constexpr int DYN_FACADE_ERR_SUBCOMPONENT    = -3399;

  HumanoidDynFacade();
  ~HumanoidDynFacade();
  HumanoidDynFacade(const HumanoidDynFacade&) = delete;
  HumanoidDynFacade& operator=(const HumanoidDynFacade&) = delete;

  // 阶段二：adapter -> Wbd/Centroidal/双手 Init + drivechain 力矩限幅缓存。
  int Init(robot_config::DynamicsConfigAdapter& adapter);
  bool IsInited() const;

  // ---- 组件直通（组合根，阶段一即可用）----
  HumanoidWbd& Wbd();
  HumanoidCentroidal& Centroidal();
  DexHandDyn& Hand(HandSide side);

  // ---- 控制前馈（热路径）----
  int ComputeFeedforward(const VecW& q, const VecW& qd, const VecW& qdd_ref,
                         const VecH& q_left, const VecH& q_right,
                         FeedforwardResult& out);

  // ---- 规划 ----
  // tau = M(q) q' sdd + [M(q) q'' sd^2 + h(q, q' sd)]
  int AccelBounds(const VecW& q, double sd,
                  const VecW& dq_ds, const VecW& d2q_ds2,
                  const VecW& tau_max, const VecW& tau_min,
                  double& sdd_min_out, double& sdd_max_out) const;
  int StabilityMargin(const VecW& q, const VecW& qd, const VecW& qdd,
                      double& margin_out) const;

  // 纯数学核心（真实实现）：tau_i = a_i sdd + b_i，
  // a = M q'，b = M q'' sd^2 + h；逐关节解 tau_min <= a sdd + b <= tau_max
  // 取交集。空交或某关节 a=0 且 b 越限时返回 INFEASIBLE。
  static int ComputeAccelBoundsCore(const MatW& m, const VecW& h,
                                    const VecW& dq_ds, const VecW& d2q_ds2,
                                    double sd,
                                    const VecW& tau_max, const VecW& tau_min,
                                    double& sdd_min_out, double& sdd_max_out);

 private:
  HumanoidWbd wbd_;
  HumanoidCentroidal centroidal_;
  DexHandDyn hand_left_;
  DexHandDyn hand_right_;
  bool inited_ = false;
};

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_FACADE_HPP_
