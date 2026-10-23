// func/dyn_humanoid/include/humanoid_wbd.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_WBD_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_WBD_HPP_

#include <cstddef>
#include <memory>

#include "dyn_humanoid_types.hpp"

namespace dyn_humanoid {

namespace detail {
struct WbdModelAccess;  // 内部访问器（src/humanoid_wbd_internal.hpp）
}

// 全身 19-DOF 固定基座整树动力学（腰4 + 头1 + 双臂14），PIMPL 隔离
// pinocchio/Eigen。阶段一为骨架：Init 返回 NOT_IMPLEMENTED；算法接口
// 未 Init 返回 NOT_INITED；链切片为真实实现（纯索引，静态，无需 Init）。
//
// 阶段二建模流程（spec §4.1）：解析 URDF → 剪枝 grip_* 夹爪分支（质量
// 聚合进 LA8/RA8）→ 校验 19 revolute 并建立索引重排表 → 可选手部集中
// 质量附加到 tool_L/tool_R。
class HumanoidWbd {
 public:
  static constexpr int HUMANOID_WBD_OK                  = 0;
  static constexpr int HUMANOID_WBD_ERR_URDF_LOAD       = -3001;
  static constexpr int HUMANOID_WBD_ERR_NOT_INITED      = -3002;
  static constexpr int HUMANOID_WBD_ERR_DOF_MISMATCH    = -3003;
  static constexpr int HUMANOID_WBD_ERR_PRUNE_FAILED    = -3004;
  static constexpr int HUMANOID_WBD_ERR_NOT_IMPLEMENTED = -3098;
  static constexpr int HUMANOID_WBD_ERR_INTERNAL        = -3099;

  HumanoidWbd();
  ~HumanoidWbd();
  HumanoidWbd(const HumanoidWbd&) = delete;
  HumanoidWbd& operator=(const HumanoidWbd&) = delete;

  // attach_hand_lump_mass=true 时把双手集中质量附加到 tool_L/tool_R。
  int Init(const char* urdf_string, size_t urdf_size, bool attach_hand_lump_mass);
  bool IsInited() const;
  int Nq() const;  // 未 Init 返回 0；Init 后返回 kWholeBodyDof

  // ---- 逆动力学：tau = M qdd + C qd + g ----
  int Rnea(const VecW& q, const VecW& qd, const VecW& qdd, VecW& tau_out) const;
  // ---- 正动力学：qdd = M^-1 (tau - C qd - g) ----
  int Aba(const VecW& q, const VecW& qd, const VecW& tau, VecW& qdd_out) const;
  // ---- 分量 ----
  int MassMatrix(const VecW& q, MatW& m_out) const;
  int BiasForce(const VecW& q, const VecW& qd, VecW& h_out) const;  // h = C qd + g
  int Gravity(const VecW& q, VecW& g_out) const;
  // ---- 运动学辅助 ----
  int FkChain(Chain chain, const VecW& q, Pose& tip_out) const;
  int JacobianChain(Chain chain, const VecW& q, Mat6xChain& j_out) const;

  // ---- 链切片（纯索引，静态，无需 Init）----
  // q_chain/qd_chain 调用方提供，容量 >= kChainDof[chain]。
  static int GetChainState(Chain chain, const VecW& q_full, const VecW& qd_full,
                           double* q_chain, double* qd_chain);
  static int SetChainState(Chain chain, const double* q_chain, const double* qd_chain,
                           VecW& q_full, VecW& qd_full);

 private:
  friend struct detail::WbdModelAccess;

  struct Impl;
  std::unique_ptr<Impl> impl_;
  bool inited_ = false;
};

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_WBD_HPP_
