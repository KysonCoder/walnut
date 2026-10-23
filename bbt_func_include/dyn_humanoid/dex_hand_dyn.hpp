// func/dyn_humanoid/include/dex_hand_dyn.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_DEX_HAND_DYN_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_DEX_HAND_DYN_HPP_

#include <memory>

#include "dyn_humanoid_types.hpp"

namespace dyn_humanoid {

// 五指灵巧手 17-DOF 生成树动力学（spec §6）：palm + thumb x5 +
// FF/MF/RF/LF x3。模型由 DexHandGeometry 程序化生成（不经 URDF 文本），
// 占位惯量可整体覆盖。
//
// 阶段一为骨架：Init 校验并存储几何（真实实现，返回 OK）；TotalMass
// 由几何直接计算（真实）；其余算法接口返回 NOT_IMPLEMENTED。
class DexHandDyn {
 public:
  static constexpr int DEX_HAND_DYN_OK                  = 0;
  static constexpr int DEX_HAND_DYN_ERR_BAD_GEOMETRY    = -3201;
  static constexpr int DEX_HAND_DYN_ERR_NOT_INITED      = -3202;
  static constexpr int DEX_HAND_DYN_ERR_BAD_FINGER_IDX  = -3203;
  static constexpr int DEX_HAND_DYN_ERR_NOT_IMPLEMENTED = -3298;

  DexHandDyn();
  ~DexHandDyn();
  DexHandDyn(const DexHandDyn&) = delete;
  DexHandDyn& operator=(const DexHandDyn&) = delete;

  // 阶段一：校验几何（finger_dof 匹配 kFingerDof、长度/半径/密度/掌质量
  // 为正）并存储，返回 OK。阶段二：pinocchio addJoint/addBody 建模。
  int Init(const DexHandGeometry& geom, HandSide side);
  bool IsInited() const;

  // ---- 集中质量描述（供 HumanoidWbd::AttachHandLumpMass）----
  double TotalMass() const;  // 几何直算；未 Init 返回 0
  int ComInPalm(const VecH& q, Vec3& com_out) const;

  // ---- 逆动力学 ----
  int Rnea(const VecH& q, const VecH& qd, const VecH& qdd, VecH& tau_out) const;
  int Gravity(const VecH& q, VecH& g_out) const;

  // ---- 指尖运动学（finger_idx ∈ [0, kNumFingers)）----
  int FingertipFk(const VecH& q, int finger_idx, Pose& tip_out) const;
  int FingertipJacobian(const VecH& q, int finger_idx, Mat6xFinger& j_out) const;

  // ---- 抓握力映射：tau = Σ_i J_i^T · f_i（逆解/优化由上层做）----
  int FingertipForcesToJointTorques(
      const VecH& q, const std::array<Vec6, kNumFingers>& f_tips,
      VecH& tau_out) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  DexHandGeometry geom_{};
  bool inited_ = false;
};

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_DEX_HAND_DYN_HPP_
