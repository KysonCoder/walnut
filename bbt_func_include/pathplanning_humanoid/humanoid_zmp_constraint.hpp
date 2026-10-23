// func/pathplanning_humanoid/include/humanoid_zmp_constraint.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_ZMP_CONSTRAINT_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_ZMP_CONSTRAINT_HPP_

#include "dyn_humanoid_types.hpp"
#include "pphumanoid_types.hpp"

namespace dyn_humanoid {
class HumanoidDynFacade;
}

namespace pphumanoid {

// ZMP 硬约束：给定 (q, q', q'', ṡ)，输出 s̈ 的可行区间使 ZMP 裕度
// ≥ safety_margin。线性化 + 端点真值核验（实现说明见 .cpp 头注释）。
class HumanoidZmpConstraint {
 public:
  struct Bounds {
    double sdd_min = 0.0;
    double sdd_max = 0.0;
    bool feasible = false;
    bool singular = false;   // ComputeZmp 报 ERR_ZMP_SINGULAR
  };

  HumanoidZmpConstraint() = default;

  // facade 必须已 Init；多边形同时写入 Centroidal（供核验用 ZmpMargin）。
  int Init(dyn_humanoid::HumanoidDynFacade* facade,
           const std::array<dyn_humanoid::Vec2, dyn_humanoid::kMaxSupportPoints>& poly,
           int num_points, double ground_z, double safety_margin);
  bool IsInited() const { return inited_; }

  int ComputeBounds(const dyn_humanoid::VecW& q,
                    const dyn_humanoid::VecW& dq_ds,
                    const dyn_humanoid::VecW& d2q_ds2,
                    double sd, Bounds& out) const;

 private:
  dyn_humanoid::HumanoidDynFacade* facade_ = nullptr;
  double safety_ = 0.0;
  bool inited_ = false;

  // 内缩半平面：n_i·z >= c_i（内缩 safety 后）
  struct HalfPlane { double nx, ny, c; };
  HalfPlane planes_[dyn_humanoid::kMaxSupportPoints];
  int num_planes_ = 0;

  // 真值裕度核验：margin(q, qd, qdd0 + dq*sdd)。奇异或失败返回 false。
  bool MarginAt_(const dyn_humanoid::VecW& q, const dyn_humanoid::VecW& qd,
                 const dyn_humanoid::VecW& qdd0, const dyn_humanoid::VecW& dq,
                 double sdd, double& margin_out) const;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_ZMP_CONSTRAINT_HPP_
