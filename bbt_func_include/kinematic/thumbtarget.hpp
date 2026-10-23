/**
 * @file thumbaction.hpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-08-27
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-08-27  0.1      Yonglin Chi       first edition
 */


#ifndef __ROBOT_THUMB_H__
#define __ROBOT_THUMB_H__


#include "basetarget.hpp"

using se3_twist::Vec5;
using se3_twist::Vec6;
using se3_twist::Vec10;
using se3_twist::Mat4;
using se3_twist::Mat8_5;

class CThumbTarget
    : public CBaseTarget
{
private:
    /* data */

    Vec6 _links_TB;
    Mat4 _links_TBse3[6];
    Vec10 TBjointlimits;

    void InitDefaultKinematics_();
    void LoadKinematicParamsFromData_();
    Mat4 RxTransform(double theta);
    Mat4 RyTransform(double theta);
    Mat4 RzTransform(double theta);
    Mat4 Tx(double distance);
    Mat4 Ty(double distance);
    Mat4 Tz(double distance);
public:

        // ... 成员变量 ...
public:
    CThumbTarget();
    explicit CThumbTarget(TargetData* data);
    ~CThumbTarget();     
    

    void SetJoints (double * joint_value) override;

    Mat4 TB_FK(Vec5 thetaVector);
    Mat8_5 TB_IK(Mat4 target_position);
    Vec5 TBfliter(Mat8_5 thetaTB, Mat4 target_position);

    int CalcPose ( const FRAME * tool_frame, const bool robot_fixed_object) override;
    int CalcJoint( const FRAME * tool_frame, const bool robot_fixed_object) override;


};

#endif