/**
 * @file joint_passthrough_target.hpp
 * @brief 无运动学模型 unit 的纯关节透传 target（2026-09-16 FR2）
 *
 * conf 显式驱动：unit 配置 kinematic_model: "joint_passthrough" 后，
 * CTarget::Attach 按模型名路由到本类。关节数据通路全部继承 CBaseTarget；
 * FK/IK 为空实现（返回 1=成功，遵循 >0 成功/0 失败/-1 未设置约定），
 * 且不置位 is_position_valid/is_orientation_valid——笛卡尔消费者仍看到
 * "位姿无效"，ACTION_LINE 端点落在透传 unit 上会在规划层硬报错。
 */

#ifndef FUNC_KINEMATIC_INCLUDE_JOINT_PASSTHROUGH_TARGET_HPP_
#define FUNC_KINEMATIC_INCLUDE_JOINT_PASSTHROUGH_TARGET_HPP_

#include "basetarget.hpp"

class CJointPassthroughTarget : public CBaseTarget {
public:
    // 必须用默认构造（_data=nullptr）：CBaseTarget(int) 会 new 私有 TargetData
    // 导致 IsAttached() 恒真，破坏 CTarget::SetUnitType 的"透传已挂？"判定。
    // 关节数不固定（BODY=4/HEAD/AGV 各异），Attach 时基类从 data->joint_number
    // 自动恢复 _joint_number_（CTargetGroup 已由 confbase unit->num_joints 填好）。
    CJointPassthroughTarget() : CBaseTarget() {}

    int CalcPose(const FRAME* tool_frame, bool robot_fixed_object) override;
    int CalcJoint(const FRAME* tool_frame, bool robot_fixed_object) override;
    int CalcJointWithConfig(const FRAME* tool_frame, bool robot_fixed_object) override;
    int CalcVelocityJacobian(float jacobian[6][6]) override;
};

#endif  // FUNC_KINEMATIC_INCLUDE_JOINT_PASSTHROUGH_TARGET_HPP_
