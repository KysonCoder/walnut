/**
 * @file humanoid_group.hpp
 * @brief Humanoid 机器人的 TargetGroup 派生类（最小骨架）。
 *
 * 提供 legacy 槽位常量与类型化访问器；初始化/数据全部由基类
 * CTargetGroup::Attach + InitializeFromRobot 驱动。
 * 与 CNieGroup（dualarm）平级，作为未来更多机器人类型的扩展样例。
 */

#ifndef HUMANOID_GROUP_HPP_
#define HUMANOID_GROUP_HPP_

#include "targetgroup.hpp"
#include "armtarget.hpp"

class CHumanoidGroup : public CTargetGroup {
 public:
  CHumanoidGroup() = default;

  static constexpr int kRightArmIndex = RIGHT_ARM;   // 0
  static constexpr int kLeftArmIndex = LEFT_ARM;     // 6
  static constexpr int kHeadIndex = ROBOT_HEAD;      // 12
  static constexpr int kBodyIndex = ROBOT_BODY;      // 13
  static constexpr int kAgvFootIndex = AGV_FOOT;     // 14

  CArmTarget& GetLeftArm();
  CArmTarget& GetRightArm();
  CTarget& GetHead();
  CTarget& GetBody();
  CTarget& GetAgvFoot();
};

#endif  // HUMANOID_GROUP_HPP_
