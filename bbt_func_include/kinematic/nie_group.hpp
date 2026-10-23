#ifndef NIE_GROUP_HPP_
#define NIE_GROUP_HPP_

#include "targetgroup.hpp"
#include "nie_waist.hpp"
#include "nie_target.hpp"
#include "armtarget.hpp"

/// @brief NIE（dualarm）机器人的 TargetGroup 派生类。
///
/// 槽位布局为紧凑枚举（kRightArmIndex=0 / kLeftArmIndex=1 / kWaistIndex=2）。
/// 槽位数据由基类 InitializeFromRobot（confbase 驱动）填充；Attach 只做
/// 校验与参数注入，不再硬填槽位。
class CNieGroup : public CTargetGroup {
 public:
  CNieGroup();

  /// @return 1 成功；ROBOT_TARGET_NOT_ATTACHED 空指针；
  ///         ROBOT_TARGET_NOT_INITIALIZED 槽位未初始化（需先 InitializeFromRobot）。
  int Attach(TargetGroupSharedData* data);
  void Detach();

  bool IsValid() const;

  CNieWaist&  GetWaist();
  CArmTarget& GetLeftArm();
  CArmTarget& GetRightArm();

  /// @return 1 成功；<0 透传子错误码（负码优先）。
  int CalcDualPose();
  int CalcDualJoint(const CArmTarget& ref_left,
                    const CArmTarget& ref_right);
  int CalcDualJacobian(double jac_left[6][6],
                       double jac_right[6][6]);

#if ROBOT_TYPE == DUAL_ARM
  static constexpr int kRightArmIndex = 0;  // DualArmSocUnitType::RIGHT_ARM
  static constexpr int kLeftArmIndex = 1;   // DualArmSocUnitType::LEFT_ARM
  static constexpr int kWaistIndex = 2;     // DualArmSocUnitType::ROBOT_BODY
#else
  static constexpr int kRightArmIndex = RIGHT_ARM;
  static constexpr int kLeftArmIndex = LEFT_ARM;
  static constexpr int kWaistIndex = ROBOT_BODY;
#endif

 private:
  interpolation::CNieTarget solver_;
  CNieWaist waist_view_;
};

#endif  // NIE_GROUP_HPP_
