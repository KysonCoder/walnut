/**
 * @file targetgroup.hpp
 * @brief
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.2
 * @date 2026-05-08
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-11-22  0.1      Yonglin Chi       first edition
 * 2026-05-08  0.2      Claude            refactor to pointer-based data, remove TARGET_GROUP_DATA
 */

#ifndef __TARGET_GROUP_HPP__
#define __TARGET_GROUP_HPP__

#include "target.hpp"
#include "joint_pos.hpp"

class CTargetGroup
{
private:
    CTarget _targets[MAX_NO_OF_SOC];
    TargetGroupSharedData * _shared_data = nullptr;

public:
    CTargetGroup();
    ~CTargetGroup();

    void Attach(TargetGroupSharedData * shared_data);
    void Detach();
    bool IsAttached() const { return _shared_data != nullptr; }

    void SetTargets(int number, CTarget * targets);
    void InitializeFromRobot();
    void SetJointPosData(const CJointPos& joint_pos);
    CTarget& GetTarget(int index);

    TargetData * GetTargetData(int soc_index);
    bool IsSocEnabled(int soc_index) const;
    void SetSocEnabled(int soc_index, bool enabled);
    TargetGroupSharedData * GetSharedData() const { return _shared_data; }

    ROBOT_STATUS UpdateRobotStatus(CJointPos & jointPos);
};


#endif // __TARGET_GROUP_HPP__
