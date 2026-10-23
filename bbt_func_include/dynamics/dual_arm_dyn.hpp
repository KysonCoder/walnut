// func/dynamics/include/dual_arm_dyn.hpp
#ifndef DUAL_ARM_DYN_HPP_
#define DUAL_ARM_DYN_HPP_

#include "math/se3_twist_ops.hpp"
#include "math/pose.hpp"
#include "dynamics_config_adapter.hpp"
#include "multiarmdyn.hpp"
#include <array>
#include <memory>
#include <string>

namespace interpolation {

using Vec6  = se3_twist::Vec6;
using Vec13 = std::array<double, 13>;

class DualArmDyn {
 public:
  static constexpr int kTotalDof = 13;
  static constexpr int kLeftArmId = 0;
  static constexpr int kRightArmId = 1;

  // Error codes
  static constexpr int DUAL_ARM_DYN_OK = 0;
  static constexpr int DUAL_ARM_DYN_ERR_URDF_LOAD = -2101;
  static constexpr int DUAL_ARM_DYN_ERR_NOT_INITED = -2102;
  static constexpr int DUAL_ARM_DYN_ERR_INTERNAL = -2199;

  DualArmDyn();
  ~DualArmDyn();

  DualArmDyn(const DualArmDyn&) = delete;
  DualArmDyn& operator=(const DualArmDyn&) = delete;

  // === Recommended: initialize from confbase ===
  int Init(const robot_config::DynamicsConfigAdapter& adapter,
           int32_t left_unit,
           int32_t right_unit,
           const std::string& base_link,
           const std::string& body_link);

  // === Legacy: kept for backward compatibility ===
  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&, ...) instead")]]
  int Init(const std::string& urdf_path,
           const std::string& base_link,
           const std::string& body_link);

  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&, ...) instead")]]
  int Init(const std::string& urdf_string,
           const std::string& base_link,
           const std::string& body_link,
           bool from_string);

  void SetBaseJointAngle(double q_base);

  // Full inverse dynamics
  int Rnea(const Vec13& q, const Vec13& qd, const Vec13& qdd,
           Vec13& tau_out) const;

  // Extract per-arm states
  void GetLeftArmState(const Vec13& q_full, const Vec13& qd_full,
                       Vec6& q_left, Vec6& qd_left) const;
  void GetRightArmState(const Vec13& q_full, const Vec13& qd_full,
                        Vec6& q_right, Vec6& qd_right) const;

  // Build full state
  void BuildFullState(double q_base,
                      const Vec6& q_left, const Vec6& qd_left,
                      const Vec6& q_right, const Vec6& qd_right,
                      Vec13& q_out, Vec13& qd_out) const;

  // Torque limit check
  bool CheckTorqueLimits(const Vec13& tau,
                         const Vec13& tau_max,
                         int& violating_joint_idx,
                         double& violation_ratio) const;

  double BaseJointTorque(const Vec13& tau) const { return tau[0]; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace interpolation

#endif // DUAL_ARM_DYN_HPP_
