// func/dynamics/include/multiarmdyn.hpp
#ifndef FUNC_DYNAMICS_INCLUDE_MULTIARMDYN_HPP_
#define FUNC_DYNAMICS_INCLUDE_MULTIARMDYN_HPP_

#include "math/se3_twist_ops.hpp"
#include "dynamics_types.hpp"
#include "dynamics_config_adapter.hpp"

#include <array>
#include <memory>

namespace urdf { class Robot; }
namespace robot_config {
struct RobotTopology;
}

namespace interpolation {

class MultiArmDyn {
 public:
  static constexpr int MULTI_ARM_DYN_OK                  = 0;
  static constexpr int MULTI_ARM_DYN_ERR_URDF_LOAD       = -2101;
  static constexpr int MULTI_ARM_DYN_ERR_NOT_INITED      = -2102;
  static constexpr int MULTI_ARM_DYN_ERR_TOPOLOGY_INVALID = -2103;
  static constexpr int MULTI_ARM_DYN_ERR_INTERNAL        = -2199;

  MultiArmDyn();
  ~MultiArmDyn();
  MultiArmDyn(const MultiArmDyn&) = delete;
  MultiArmDyn& operator=(const MultiArmDyn&) = delete;

  // Recommended: fully automatic from confbase RobotTopology.
  int Init(const robot_config::DynamicsConfigAdapter& adapter);

  // Legacy: kept for backward compatibility.
  [[deprecated("Use Init(const robot_config::DynamicsConfigAdapter&) instead")]]
  int Init(const std::string& urdf_string,
           const std::string& global_base_link,
           const robot_config::RobotTopology& topology);

  int Nq() const;
  int GetPrefixDof() const;

  int Rnea(const VecR& q, const VecR& qd, const VecR& qdd,
           VecR& tau_out) const;

  int GetChainCount() const;
  int GetChainDof(int chain_index) const;
  int GetChainOffset(int chain_index) const;

  // Chain state includes the shared prefix followed by the chain-specific
  // suffix. |q_chain| must have at least GetChainDof(chain_index) elements.
  int GetChainState(int chain_index,
                    const VecR& q_full, const VecR& qd_full,
                    VecA& q_chain, VecA& qd_chain) const;
  int SetChainState(int chain_index,
                    const VecA& q_chain, const VecA& qd_chain,
                    VecR& q_full, VecR& qd_full) const;

  bool CheckTorqueLimits(const VecR& tau,
                         const VecR& tau_max,
                         int& violating_joint_idx,
                         double& violation_ratio) const;
  double BaseJointTorque(const VecR& tau) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  int InitFromTopology_(const urdf::Robot& urdf_robot,
                        const std::string& global_base_link,
                        const robot_config::RobotTopology& topology);
};

} // namespace interpolation

#endif // FUNC_DYNAMICS_INCLUDE_MULTIARMDYN_HPP_
