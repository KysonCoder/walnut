#ifndef SPLINE_TYPES_HPP_
#define SPLINE_TYPES_HPP_

#include "math/masked_array_operations.h"
#include "confbase/robot.hpp"  // for robot_config::kRobotMaxJoints

namespace spline {

#if ROBOT_TYPE == HUMANOID
static_assert(robot_config::kRobotMaxJoints == 57,
              "JointArray size is pinned to 57 joints (HUMANOID)");
#elif ROBOT_TYPE == DUAL_ARM
static_assert(robot_config::kRobotMaxJoints == 13,
              "JointArray size is pinned to 13 joints (DUAL_ARM)");
#endif

// 关节数组类型 - 在样条系统中取代CJointd
using JointArray = MaskedArray<double, robot_config::kRobotMaxJoints>;

// 前向声明
class ActionRef;      // CAction的轻量级包装
struct SpeedCondition; // 速度限制条件

} // namespace spline

#endif // SPLINE_TYPES_HPP_
