// func/dynamics/include/GenrArmDyn.hpp
#ifndef GENR_ARMDYN_HPP_
#define GENR_ARMDYN_HPP_

#include "math/se3_twist_ops.hpp"
#include "math/pose.hpp"
#include "dynamics_config_adapter.hpp"
#include <array>
#include <memory>
#include <string>

namespace urdf { class Robot; }  // forward declare

namespace interpolation {

using Vec3  = se3_twist::Vec3;
using Vec6  = se3_twist::Vec6;
using Mat3  = se3_twist::Mat3;
using Mat6  = se3_twist::Mat6;
using Twist = se3_twist::Twist;
using Pose  = math::Pose<double>;

class ArmDyn;  // forward

class GenrArmDyn {
 public:
  // Error codes
  static constexpr int GENR_ARM_DYN_OK                  = 0;
  static constexpr int GENR_ARM_DYN_ERR_URDF_LOAD       = -2001;
  static constexpr int GENR_ARM_DYN_ERR_ROOT_NOT_FOUND  = -2002;
  static constexpr int GENR_ARM_DYN_ERR_TIP_NOT_FOUND   = -2003;
  static constexpr int GENR_ARM_DYN_ERR_NO_PATH         = -2004;
  static constexpr int GENR_ARM_DYN_ERR_BAD_JOINT_TYPE  = -2005;
  static constexpr int GENR_ARM_DYN_ERR_BAD_INERTIAL    = -2006;
  static constexpr int GENR_ARM_DYN_ERR_DOF_NOT_6       = -2007;
  static constexpr int GENR_ARM_DYN_ERR_NOT_INITED      = -2008;
  static constexpr int GENR_ARM_DYN_ERR_INTERNAL        = -2099;

  GenrArmDyn();
  ~GenrArmDyn();

  GenrArmDyn(const GenrArmDyn&) = delete;
  GenrArmDyn& operator=(const GenrArmDyn&) = delete;

  // ---- Initialization ----
  // === Recommended: initialize from confbase ===
  int Init(const robot_config::DynamicsConfigAdapter& adapter,
           int32_t unit,
           const std::string& root_link,
           const std::string& tip_link);

  // === Legacy: kept for backward compatibility ===
  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&, ...) instead")]]
  int Init(const std::string& urdf_path,
           const std::string& root_link,
           const std::string& tip_link);

  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&, ...) instead")]]
  int Init(const std::string& urdf_string,
           const std::string& root_link,
           const std::string& tip_string,
           bool from_string);

  void SetGravity(const Vec3& g_in_root);

  // ---- Metadata ----
  int Nq() const;
  const std::array<std::string, 6>& JointNames() const;
  void JointLimits(Vec6& lower_out, Vec6& upper_out) const;

  // ---- Inverse dynamics: tau = M qdd + C qd + g ----
  int Rnea(const Vec6& q, const Vec6& qd, const Vec6& qdd,
           Vec6& tau_out) const;

  // ---- Forward dynamics: qdd = M^-1 (tau - C qd - g) ----
  int Aba(const Vec6& q, const Vec6& qd, const Vec6& tau,
          Vec6& qdd_out) const;

  // ---- Components ----
  int MassMatrix(const Vec6& q, Mat6& M_out) const;
  int Gravity(const Vec6& q, Vec6& g_out) const;
  int CoriolisTimesQd(const Vec6& q, const Vec6& qd,
                      Vec6& Cqd_out) const;
  int CoriolisMatrix(const Vec6& q, const Vec6& qd,
                     Mat6& C_out) const;

  // ---- Kinematics helpers ----
  int Fk(const Vec6& q, Pose& tip_in_root_out) const;
  int Jacobian(const Vec6& q, Mat6& J_out) const;
  int TipTwist(const Vec6& q, const Vec6& qd, Twist& v_out) const;

  // ---- Tool / payload (lumped at tip) ----
  int SetTool(double mass,
              const Vec3& com_in_tip,
              const Mat3& inertia_in_tip);
  void ClearTool();

  // ---- Joint friction ----
  int SetJointDamping(const Vec6& b);
  int SetJointCoulombFriction(const Vec6& fc);
  void ClearJointFriction();

  // ---- Persistent tip external wrench ----
  void SetTipExternalWrench(const Vec6& wrench);
  void ClearExternalWrenches();

  // ---- Explicit one-shot variants ----
  int RneaWithExtWrench(const Vec6& q, const Vec6& qd, const Vec6& qdd,
                        const Vec6& tip_wrench_in_root,
                        Vec6& tau_out) const;
  int AbaWithExtWrench(const Vec6& q, const Vec6& qd, const Vec6& tau,
                       const Vec6& tip_wrench_in_root,
                       Vec6& qdd_out) const;

  static const char* Version();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  void SyncJointNames_();
};

} // namespace interpolation

#endif // GENR_ARMDYN_HPP_
