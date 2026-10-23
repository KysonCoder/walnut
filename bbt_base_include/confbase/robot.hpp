/**
 * @file robot.hpp
 * @brief
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-08-22
 *
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-08-22  0.1      Yonglin Chi       first edition
 */

#ifndef BASE_CONFBASE_INCLUDE_ROBOT_HPP_
#define BASE_CONFBASE_INCLUDE_ROBOT_HPP_

#ifndef HUMANOID
#define HUMANOID 1
#endif
#ifndef DUAL_ARM
#define DUAL_ARM 2
#endif

#ifndef ROBOT_TYPE
#error "ROBOT_TYPE must be defined explicitly (-DROBOT_TYPE=HUMANOID or -DROBOT_TYPE=DUAL_ARM)"
#endif

#include "joint_pos.hpp"

#if ROBOT_TYPE == HUMANOID
#include "humanoid_conf.hpp"
#include "humanoid_config_builder.hpp"
#include "humanoid_config_store.hpp"
#elif ROBOT_TYPE == DUAL_ARM
#include "dualarm_conf.hpp"
#include "dualarm_config_builder.hpp"
#include "dualarm_config_store.hpp"
#else
#error "Unsupported ROBOT_TYPE. Expected HUMANOID(1) or DUAL_ARM(2)."
#endif

namespace robot_config {

#if ROBOT_TYPE == HUMANOID
  using RobotConfigBuilder = HumanoidConfigBuilder;
  using RobotConfigStore   = HumanoidConfigStore;
  using RobotSocUnitType   = HumanoidSocUnitType;
  constexpr uint32_t kRobotMaxJoints     = kHumanoidMaxJoints;
  constexpr uint32_t kRobotMaxUnits      = kHumanoidMaxUnits;
  constexpr uint32_t kRobotMaxUnitJoints = kHumanoidMaxUnitJoints;
  constexpr uint32_t kRobotMaxUnitLinks  = kHumanoidMaxUnitLinks;
#else
  using RobotConfigBuilder = DualArmConfigBuilder;
  using RobotConfigStore   = DualArmConfigStore;
  using RobotSocUnitType   = DualArmSocUnitType;
  constexpr uint32_t kRobotMaxJoints     = kDualArmMaxJoints;
  constexpr uint32_t kRobotMaxUnits      = kDualArmMaxUnits;
  constexpr uint32_t kRobotMaxUnitJoints = kDualArmMaxUnitJoints;
  constexpr uint32_t kRobotMaxUnitLinks  = kDualArmMaxUnitLinks;
#endif

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_ROBOT_HPP_
