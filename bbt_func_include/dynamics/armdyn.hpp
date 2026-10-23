// func/dynamics/include/armdyn.hpp
#ifndef FUNC_DYNAMICS_INCLUDE_ARMDYN_HPP_
#define FUNC_DYNAMICS_INCLUDE_ARMDYN_HPP_

#include <cstdint>

#include "math/se3_twist_ops.hpp"
#include "math/pose.hpp"
#include "dynamics_types.hpp"
#include "dynamics_config_adapter.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace urdf { struct Robot; }

namespace interpolation {

using Vec3 = se3_twist::Vec3;
using Vec6 = se3_twist::Vec6;
using Mat3 = se3_twist::Mat3;
using Twist = se3_twist::Twist;
using Pose = math::Pose<double>;

class ArmDyn {
 public:
  static constexpr int ARM_DYN_OK                  = 0;
  static constexpr int ARM_DYN_ERR_URDF_LOAD       = -2001;
  static constexpr int ARM_DYN_ERR_ROOT_NOT_FOUND  = -2002;
  static constexpr int ARM_DYN_ERR_TIP_NOT_FOUND   = -2003;
  static constexpr int ARM_DYN_ERR_NO_PATH         = -2004;
  static constexpr int ARM_DYN_ERR_BAD_JOINT_TYPE  = -2005;
  static constexpr int ARM_DYN_ERR_BAD_INERTIAL    = -2006;
  static constexpr int ARM_DYN_ERR_DOF_EXCEEDS_MAX = -2007;
  static constexpr int ARM_DYN_ERR_NOT_INITED      = -2008;
  static constexpr int ARM_DYN_ERR_INTERNAL        = -2099;

  ArmDyn();
  ~ArmDyn();
  ArmDyn(const ArmDyn&) = delete;
  ArmDyn& operator=(const ArmDyn&) = delete;

  bool IsInited() const;

  // Recommended: fully automatic from confbase.
  int Init(const robot_config::DynamicsConfigAdapter& adapter,
           int32_t unit_type);

  // Legacy: kept for backward compatibility.
  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&, int32_t) instead")]]
  int Init(const std::string& urdf_path,
           const std::string& root_link,
           const std::string& tip_link);

  [[deprecated]]
  int Init(const std::string& urdf_string,
           const std::string& root_link,
           const std::string& tip_link,
           bool /*from_string*/);

  void SetGravity(const Vec3& g_in_root);

  // Metadata
  int Nq() const;
  const std::vector<std::string>& JointNames() const;
  void JointLimits(VecA& lower_out, VecA& upper_out) const;

  // Dynamics
  int Rnea(const VecA& q, const VecA& qd, const VecA& qdd,
           VecA& tau_out) const;
  int Aba(const VecA& q, const VecA& qd, const VecA& tau,
          VecA& qdd_out) const;
  int MassMatrix(const VecA& q, MatAxA& M_out) const;
  int Gravity(const VecA& q, VecA& g_out) const;
  int CoriolisTimesQd(const VecA& q, const VecA& qd,
                      VecA& Cqd_out) const;
  int CoriolisMatrix(const VecA& q, const VecA& qd,
                     MatAxA& C_out) const;

  int RneaWithExtWrench(const VecA& q, const VecA& qd, const VecA& qdd,
                        const Vec6& tip_wrench_in_root,
                        VecA& tau_out) const;
  int AbaWithExtWrench(const VecA& q, const VecA& qd, const VecA& tau,
                       const Vec6& tip_wrench_in_root,
                       VecA& qdd_out) const;

  // Kinematics
  int Fk(const VecA& q, Pose& tip_in_root_out) const;
  int Jacobian(const VecA& q, Mat6xA& J_out) const;
  int TipTwist(const VecA& q, const VecA& qd, Twist& v_out) const;

  // Tool / payload (lumped at tip)
  int SetTool(double mass, const Vec3& com_in_tip,
              const Mat3& inertia_in_tip);
  void ClearTool();

  // Joint friction
  int SetJointDamping(const VecA& b);
  int SetJointCoulombFriction(const VecA& fc);
  void ClearJointFriction();

  // Persistent tip external wrench
  void SetTipExternalWrench(const Vec6& wrench);
  void ClearExternalWrenches();

  static const char* Version();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  int InitFromUrdfRobot_(const urdf::Robot& urdf_robot,
                         const std::string& root_link,
                         const std::string& tip_link);
  void AddFriction_(const VecA& qd, VecA& tau) const;
  void SubtractFriction_(const VecA& qd, VecA& tau) const;
  void BuildFext_(const VecA& q, const Vec6& wrench_in_root,
                  void* fext_void) const;
};

} // namespace interpolation

#endif // FUNC_DYNAMICS_INCLUDE_ARMDYN_HPP_
