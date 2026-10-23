// func/dynamics/include/dynamics_config_adapter.hpp
#ifndef FUNC_DYNAMICS_INCLUDE_DYNAMICS_CONFIG_ADAPTER_HPP_
#define FUNC_DYNAMICS_INCLUDE_DYNAMICS_CONFIG_ADAPTER_HPP_

#include "confbase/dynamics_config_adapter.hpp"

namespace dynamics {

// Deprecated alias: func/dynamics no longer maintains its own wrapper.
// confbase now provides typed Attach() overloads for DualArmConfigStore and
// HumanoidConfigStore, so the robot_config::DynamicsConfigAdapter can be used
// directly. This alias is kept for backward compatibility.
using DynamicsConfigAdapter [[deprecated(
    "Use robot_config::DynamicsConfigAdapter directly")]] =
    robot_config::DynamicsConfigAdapter;

}  // namespace dynamics

#endif  // FUNC_DYNAMICS_INCLUDE_DYNAMICS_CONFIG_ADAPTER_HPP_
