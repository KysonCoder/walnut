// func/dyn_humanoid/include/humanoid_centroidal.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_CENTROIDAL_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_CENTROIDAL_HPP_

#include <array>
#include <memory>

#include "dyn_humanoid_types.hpp"

namespace dyn_humanoid {

class HumanoidWbd;

// 质心动力学（spec §5）：总质心、质心动量及导数、ZMP、支撑多边形裕度。
// 固定基座语义：世界系 = Base 系，多边形顶点与 ZMP 均在 Base 系地面
// 投影平面（米）。
//
// 阶段一为骨架：SetSupportPolygon / ZmpMargin 为真实实现（纯几何，
// 无需 Init）；Compute* 与 Init 待阶段二 pinocchio 接线。
class HumanoidCentroidal {
 public:
  static constexpr int HUMANOID_CENTROIDAL_OK                  = 0;
  static constexpr int HUMANOID_CENTROIDAL_ERR_NOT_INITED      = -3101;
  static constexpr int HUMANOID_CENTROIDAL_ERR_NO_SUPPORT_POLY = -3102;
  static constexpr int HUMANOID_CENTROIDAL_ERR_ZMP_SINGULAR    = -3103;
  static constexpr int HUMANOID_CENTROIDAL_ERR_BAD_POLYGON     = -3104;
  static constexpr int HUMANOID_CENTROIDAL_ERR_NOT_IMPLEMENTED = -3198;

  HumanoidCentroidal();
  ~HumanoidCentroidal();
  HumanoidCentroidal(const HumanoidCentroidal&) = delete;
  HumanoidCentroidal& operator=(const HumanoidCentroidal&) = delete;

  // 绑定 Wbd 的模型（只读借用，不拥有）。wbd 未 Init 时返回 NOT_INITED。
  int Init(const HumanoidWbd& wbd);
  bool IsInited() const;

  // ---- 支撑多边形（纯几何，无需 Init）----
  // points_base: Base 系地面投影平面上的接触点（米），num_points ∈
  // [3, kMaxSupportPoints]；Set 时凸包化（非凸输入自动取凸包）。
  // ground_z: 地面在 Base 系下的 z。不提供隐式默认值——未设置时
  // ZMP 接口返回 NO_SUPPORT_POLY。可重复调用（动态支撑域）。
  int SetSupportPolygon(const std::array<Vec2, kMaxSupportPoints>& points_base,
                        int num_points, double ground_z);
  bool HasSupportPolygon() const;

  // 裕度 = ZMP 到多边形各边的最小有向距离（内正外负）。
  int ZmpMargin(const Vec2& zmp_base, double& margin_out) const;

  // ---- 质心动力学（阶段二实现）----
  int ComputeCom(const VecW& q, Vec3& com_out) const;
  int ComputeComVelocity(const VecW& q, const VecW& qd, Vec3& com_vel_out) const;
  int ComputeCentroidalMomentum(const VecW& q, const VecW& qd,
                                Vec3& lin_out, Vec3& ang_out) const;
  int ComputeCentroidalMomentumDot(const VecW& q, const VecW& qd, const VecW& qdd,
                                   Vec3& lin_dot_out, Vec3& ang_dot_out) const;
  // p_zmp = c_xy - (c_z - ground_z)/(l_dot_z/m + g) * (l_dot_xy/m)
  // l_dot_z/m + g ≈ 0（离地）时返回 ZMP_SINGULAR。
  int ComputeZmp(const VecW& q, const VecW& qd, const VecW& qdd,
                 Vec2& zmp_base_out) const;
  int CheckStability(const VecW& q, const VecW& qd, const VecW& qdd,
                     double safety_margin, bool& stable_out,
                     double& margin_out) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  std::array<Vec2, kMaxSupportPoints> poly_{};  // 凸包化后的 CCW 顶点
  int poly_count_ = 0;
  double ground_z_ = 0.0;
  bool inited_ = false;
};

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_CENTROIDAL_HPP_
