/**
 * @file fingeraction.hpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-08-23
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-08-23  0.1      Yonglin Chi       first edition
 */



#ifndef __ROBOT_FINGER_ACTION_H__
#define __ROBOT_FINGER_ACTION_H__

#include "basetarget.hpp"

using se3_twist::Vec4;
using se3_twist::Vec8;
using se3_twist::Mat4;

extern int global_finger_num;  
class CGFingerTarget
    : public CBaseTarget
{
private:
    /* data */

    double _k_factor;
    
    Vec4 _links_a;
    Mat4 _links_se3[5];
    Vec8 jointlimits;

    double TriangleFunction(double x);

    double FindRoot(double left, double right, double tolerance);

    // Eigen::Matrix4d dhTransform(Eigen::Vector4d DH_PARAMS);
    Mat4 RxTransform(double theta);
    Mat4 RyTransform(double theta);
    Mat4 RzTransform(double theta);
    Mat4 Tx(double distance);
    Mat4 Ty(double distance);
    Mat4 Tz(double distance);

    void InitDefaultKinematics_();
    void LoadKinematicParamsFromData_();
public:
    

    CGFingerTarget();
    explicit CGFingerTarget(TargetData* data);

    ~CGFingerTarget();

    void SetJoints (double * joint_value) override;


    Mat4 finger();
    // Eigen::Matrix4d FK(Eigen::Vector4d DH_1, Eigen::Vector4d DH_2,Eigen::Vector4d DH_3,Eigen::Vector4d DH_4,Eigen::Vector4d DH_5) ;
    Mat4 FK(Vec4 thetaVector);
    Mat4 IK(Mat4 target_position);
    Vec4 fliter(Mat4 theta, Mat4 target_position);


    int CalcPose ( const FRAME * tool_frame,const bool robot_fixed_object) override;


    int CalcJointWithConfig (const FRAME * tool_frame,const bool robot_fixed_object) override;

    int CalcJoint( const FRAME * tool_frame,const bool robot_fixed_object) override;

    int CalcVelocityJacobian ( float jacobian[6][6]) override;


    
};









#endif