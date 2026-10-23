#ifndef SERVO_CROBOT_TRANSFORMER_HPP_
#define SERVO_CROBOT_TRANSFORMER_HPP_

#include "robot_transformer.hpp"

namespace servo {

// 基于 RobotConfigStore（新 confbase）的 RobotTransformer 实现。
// 无状态；调用时经 RobotConfigStore::Instance() 取配置。
class CRobotTransformer : public RobotTransformer {
public:
    CRobotTransformer() = default;

    bool JointToMotor(
        const CJointPos& joint_pos,
        const std::array<bool, robot_config::kRobotMaxJoints>& mask,
        CJointPos& motor_pos) const override;

    bool GetUnitJointContext(
        robot_config::RobotSocUnitType unit_type,
        const robot_config::UnitConfig*& unit_config,
        const robot_config::GlobalJointMap*& global_map,
        const robot_config::GlobalJointParams*& joint_params) const override;
};

}  // namespace servo

#endif
