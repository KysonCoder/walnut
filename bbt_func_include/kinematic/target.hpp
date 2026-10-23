/**
 * @file target.hpp
 * @brief
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.2
 * @date 2026-05-08
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-11-13  0.1      Yonglin Chi       first edition
 * 2026-05-07  0.2      Claude            replace CGArmTarget with CArmTarget, connect kinematic methods
 * 2026-05-08  0.3      Claude            refactor to pointer-based data, remove TARGET_DATA
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include "pose.hpp"
#include "armtarget.hpp"
#include "gfingertarget.hpp"
#include "thumbtarget.hpp"
#include "joint_passthrough_target.hpp"

using interpolation::CArmTarget;


class CTarget
{
private:
    CArmTarget _arm_target;
    CGFingerTarget _gfinger_target;
    CThumbTarget _thumb_target;
    // 2026-09-16 FR2: 纯关节透传（模型名 "joint_passthrough" 时路由）
    CJointPassthroughTarget _passthrough_target;

    CBaseTarget * _active_target;

    SOC_UNIT_TYPE _unit_type;

    // 2026-09-27：值初始化清零——ResolveRobotParam_ 分支谓词 no_of_joints>0，
    // 零 = 确定性走 conf/URDF 回退（修复前为未初始化垃圾，task 侧 valgrind 实证，
    // docs/2026-09-24-func-armtarget-uninit-robot-param-defect.md）。
    MAN_ROBOT _robot_param{};

    /// Kinematic context for automatic synchronization.
    const FRAME * _tool_frame = nullptr;
    bool _robot_fixed_object = false;
    bool _auto_sync_enabled = false;

    /// Internal helpers for auto-sync.
    void SyncPoseFromJoints_();
    void SyncJointsFromPose_();

public:

    CTarget();
    CTarget(const CTarget& other);
    CTarget & operator = (const CTarget & other);

    void Attach(SOC_UNIT_TYPE unit_type, TargetData * data);
    void Detach();
    bool IsAttached() const;

    void ResetPoseFlag();
    void ResetJointsFlag();

    double GetJointSpeedLimit();
    double GetJointAccLimit();

    TARGET_TYPE GetTargetType();

    SOC_UNIT_TYPE GetUnitType() { return _unit_type; }
    bool SetUnitType(SOC_UNIT_TYPE unit_type);

    // 2026-09-02 R1: 端点插补类型访问器（转发到 _active_target，
    // 读写共享 TargetData::motion_type；未 Attach 时读为 ACTION_NULL、写忽略）
    void SetMotionType(ACTION_MOTION_TYPE motion_type);
    ACTION_MOTION_TYPE GetMotionType() const;

    int GetJointNumber();

    CArmTarget & GetArmTarget() { return _arm_target; }
    CGFingerTarget & GetGfingerTarget() { return _gfinger_target; }
    CThumbTarget & GetThumbTarget() { return _thumb_target; }
    CJointPassthroughTarget & GetPassthroughTarget() { return _passthrough_target; }

    // 2026-09-16 FR2: 类型无关的共享数据访问——返回当前路由 target 的
    // TargetData（透传 target 时 GetArmTarget().GetData() 拿到的是 arm 成员
    // 的私有零值副本，禁止再用于通用数据拷贝）。未 Attach 返回 nullptr。
    TargetData* GetActiveTargetData() {
        return _active_target ? _active_target->GetData() : nullptr;
    }
    const TargetData* GetActiveTargetData() const {
        return _active_target ? _active_target->GetData() : nullptr;
    }

    void SetArmTarget(SOC_UNIT_TYPE unit_type, CArmTarget & arm_target);
    void SetGfingerTarget(SOC_UNIT_TYPE unit_type, CGFingerTarget & gfinger_target);
    void SetThumbTarget(SOC_UNIT_TYPE unit_type, CThumbTarget & thumb_target);

    void SetRobotParam(const MAN_ROBOT & robot_param) { _robot_param = robot_param; }
    const MAN_ROBOT & GetRobotParam() const { return _robot_param; }

    ///@name Automatic kinematic synchronization
    ///@{

    /// Enable automatic FK/IK synchronization.
    /// When enabled, SetJoints() automatically calls CalcPoseNom() to update
    /// Cartesian pose from joints, and SetPose() automatically calls CalcJoint()
    /// to update joints from Cartesian pose (only if joints are already valid).
    void EnableAutoSync(const FRAME * tool_frame, bool robot_fixed_object);
    void DisableAutoSync();
    bool IsAutoSyncEnabled() const { return _auto_sync_enabled; }

    ///@}

    ~CTarget();

    void SetName(const char * name);
    void GetName(char * name);

    void SetPose(const math::Cartesian3<double> position, const math::Quaternion<double> orientation);
    void SetPose(SE3_MATRIX se3, const TARGET_CONFIG config);
    void GetPose(SE3_MATRIX * se3, TARGET_CONFIG * config);
    void GetPose(math::Cartesian3<double> * position, math::Quaternion<double> * orientation, TARGET_CONFIG * config);

    void SetSocJoints(const SOC_JOINTS joints_value);
    void SetJoints(double * joint_value);
    void GetJoints(SOC_JOINTS * joints_value) const;

    double GetJoint(int joint_no);
    void SetConfig(const TARGET_CONFIG config_in);
    math::Cartesian3<double> GetPosition() const;
    math::Quaternion<double> GetOrientation() const;

    bool IsJointsValid() const;
    bool IsPoseValid() const;

    void SetPosition(const math::Cartesian3<double> position);
    void SetOrientation(const math::Quaternion<double> orientation);

    void GetArmAngle(float *arm_angle_in);
    void SetArmAngle(const float * arm_angle_in);

    int CalcPose(const FRAME * tool_frame, const bool robot_fixed_object);
    int CalcJointWithConfig(const FRAME * tool_frame, const bool robot_fixed_object);
    int CalcJoint(const FRAME * tool_frame, const bool robot_fixed_object);
    int CalcVelocityJacobian(float jacobian[6][6]);
};


#endif // __TARGET_HPP__
