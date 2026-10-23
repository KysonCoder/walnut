#ifndef SERVO_ROBOT_TRANSFORMER_HPP_
#define SERVO_ROBOT_TRANSFORMER_HPP_

#include <array>
#include "confbase/robot.hpp"
#include "confbase/joint_pos.hpp"

namespace servo {

class RobotTransformer {
public:
    virtual ~RobotTransformer() = default;

    // joint_pos/motor_pos 必须带 GlobalJointMap/GlobalJointParams 构造
    // （经 GetUnitJointContext 获取），否则逻辑映射与电机转换不可用。
    virtual bool JointToMotor(
        const CJointPos& joint_pos,
        const std::array<bool, robot_config::kRobotMaxJoints>& mask,
        CJointPos& motor_pos) const = 0;

    // 输出 unit 配置与全局关节映射/参数（均为 RobotConfigStore 持有的
    // shm 指针，调用方不得释放）。Store 未 attach 或 unit 不存在返回 false。
    virtual bool GetUnitJointContext(
        robot_config::RobotSocUnitType unit_type,
        const robot_config::UnitConfig*& unit_config,
        const robot_config::GlobalJointMap*& global_map,
        const robot_config::GlobalJointParams*& joint_params) const = 0;
};

}  // namespace servo

#endif
