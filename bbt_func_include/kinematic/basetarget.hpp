/**
 * @file basetarget.hpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-10-30
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-10-30  0.1      Yonglin Chi       first edition
 */

#ifndef __BASE_TARGET_H__
#define __BASE_TARGET_H__


#include "logger.h"

#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "pose.hpp"
#include "se3_twist_ops.hpp"

#include "confbase/robot_config_common.hpp"
#include "legacy_robot_types.hpp"

#define SOC_INSTANCE_NAME_LEN 32
#define SOC_MAX_JOINT_NUMBER 7

typedef se3_twist::Mat4 SE3_MATRIX;

///@name Singularity and warning codes
///@{
#define ROBOT_TARGET_ARM_WCP_SINGULARITY                        10
#define ROBOT_TARGET_WRIST_SINGULARITY                          11
#define ROBOT_TARGET_ARM_WCP_WRIST_SINGULARITY                  12
#define ROBOT_TARGET_ARM_TCP_SINGULARITY                        13
#define ROBOT_TARGET_JOINT_MOTION_WARNING                       14
#define ROBOT_TARGET_ARM_WCP_SINGULARITY_JOINT_MOTION_WARNING   15
#define ROBOT_TARGET_WRIST_SINGULARITY_JOINT_MOTION_WARNING     16
#define ROBOT_TARGET_ARM_WCP_WRIST_SINGULARITY_JOINT_MOTION_WARNING 17
#define ROBOT_TARGET_ARM_TCP_SINGULARITY_JOINT_MOTION_WARNING   18
#define ROBOT_TARGET_QUATERNION_NOT_NORMALIZED                  20
///@brief 19 保留。返回值约定：1=OK，正=warning，负=error；
/// 0 为非法/未初始化哨兵——任何求解路径不得返回 0（2026-10-10 起）。
///@}

///@name Error codes
///@{
#define ROBOT_TARGET_POSE_OUTSIDE_REACH                       -101
#define ROBOT_TARGET_ORIENTATION_OUTSIDE_REACH                -102
#define ROBOT_TARGET_POSE_CONFIG_INCOMPATABLE                 -124
#define ROBOT_TARGET_JOINT_MOTION_ERROR                       -155
#define ROBOT_TARGET_WEAVE_POSE_DEFINITION                    -200
#define ROBOT_TARGET_WRIST_WEAVE_DEFINITION                   -201
#define ROBOT_TARGET_AXIS_1_ERROR                            -301
#define ROBOT_TARGET_AXIS_2_ERROR                            -302
#define ROBOT_TARGET_AXIS_3_ERROR                            -303
#define ROBOT_TARGET_AXIS_4_ERROR                            -304
#define ROBOT_TARGET_AXIS_5_ERROR                            -305
#define ROBOT_TARGET_AXIS_6_ERROR                            -306
///@}

///@name Lifecycle and limit error codes
///@{
#define ROBOT_TARGET_NOT_ATTACHED                              -1
#define ROBOT_TARGET_NOT_INITIALIZED                         -107
#define ROBOT_TARGET_JOINT_OUT_OF_RANGE                      -108
///@}

///@name Solver detail error codes（与 IRB 家族数值对齐，ELBOW 在公共边界翻译至此）
///@{
#define ROBOT_TARGET_JOINT_4_UNCERTAIN                       -125
#define ROBOT_TARGET_JOINT_6_UNCERTAIN                       -126
#define ROBOT_TARGET_VALID_JOINTS_NOT_FOUND                  -127
#define ROBOT_TARGET_ERROR_MODEL_NOT_IMPL                    -203
///@}

///@name Reach failure sub-causes（2026-10-10：遗产细分粒度恢复）
///@{
#define ROBOT_TARGET_REACH_J1_NO_SOLUTION                    -111
#define ROBOT_TARGET_REACH_J2_NO_SOLUTION                    -112
#define ROBOT_TARGET_REACH_ELBOW_TRIANGLE                    -113
#define ROBOT_TARGET_REACH_STAGE4_NO_SOLUTION                -114
#define ROBOT_TARGET_REACH_J23_VERIFY_FAILED                 -115
///@}
///@brief -114 备注：肘式遗产 REACH_4(-1004) 是定义未用死码；IRB 平行连杆的
/// 第 4 级（IRB_TARGET_POSE_OUTSIDE_REACH_4）在用——本号服务 IRB 分支。

typedef struct
{
   int quarter_rev_j1;
   int quarter_rev_j4;
   int quarter_rev_j6;
   int quarter_rev_jx;
}
TARGET_CONFIG;

///@brief POD data structure for all target shared data.
/// Contains fields from CBaseTarget and CArmTarget (shared across processes).
struct TargetData {
    // CBaseTarget common fields
    char name[SOC_INSTANCE_NAME_LEN + 1];
    SOC_UNIT_TYPE unit_type;
    POSE pose;
    TARGET_CONFIG config;
    double joints[SOC_MAX_JOINT_NUMBER];
    int joint_number;
    bool is_position_valid;
    bool is_orientation_valid;
    bool is_joints_valid;
    bool elog_enable;

    // CArmTarget specific fields
    char arm_name[SOC_INSTANCE_NAME_LEN + 1];
    double arm_angle;
    int robot_type;
    double cp_arm[3];
    double cp_wcp[3];

    // New: kinematic model selection + params (POD, shared-memory safe)
    // root_link / tip_link are NOT stored here; consumed from confbase at init time (Option A)
    char kinematic_model_name[64];
    robot_config::JointKinematicParams kinematic_params[SOC_MAX_JOINT_NUMBER];

    // 2026-09-02 R1: 本端点作为动作终点的插补类型。
    //  - ACTION_NULL(0)：本 TargetGroup 中该 SOC 目标点为空——对应 action 不规划、
    //    不下发伺服指令；序列中段 NULL = 保持位置（该段跳过）
    //  - ACTION_LINE / ACTION_JOINT：到达本端点的动作类型
    //  - 其余值（CIRCLE/SPLINE/FORCE/SEARCH）：暂不支持，编排层硬报错
    // 注意：零初始化（memset 0）= ACTION_NULL = 无运动。调用方必须显式填写。
    ACTION_MOTION_TYPE motion_type;
};

///@brief Shared data for CTargetGroup (cross-process shared).
struct TargetGroupSharedData {
    TargetData targets[MAX_NO_OF_SOC];
    bool soc_enable[MAX_NO_OF_SOC];
};

enum point_ipol_method
{
   point_line,
   position_joint_123,
   position_joint_456,
   position_joint_156,
   position_joint_134,
   position_joint_567
};



enum frame_coord_type
{
   FRAME_COORD_TO_NONE,
   FRAME_COORD_TO_ROBOT_BASE,
   FRAME_COORD_TO_WORLD
};



/// 2026-10-10 报告 §3.7：工况日志按 target 数据门控（遗产 elog_enable 复活）。
/// 契约违反类错误（未 Attach/未初始化）不走此门控，保持强制 Log_E。
/// 热路径合规：栈缓冲 + 无堆分配。
inline void KinematicLogIfEnabled(const TargetData* data, const char* fmt, ...) {
  if (data == nullptr || !data->elog_enable) return;
  va_list ap;
  va_start(ap, fmt);
  char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Log_E("%s", buf);
}

class CBaseTarget
{

protected:
   TargetData* _data = nullptr;
   bool _owns_data = false;

   int _joint_number_;

public:

   CBaseTarget(int joint_number);
   explicit CBaseTarget(TargetData* data);
   CBaseTarget();
   CBaseTarget(const CBaseTarget& other);
   CBaseTarget& operator=(const CBaseTarget& other);

   ~CBaseTarget();

   void Attach(TargetData* data);
   void Detach();
   bool IsAttached() const { return _data != nullptr; }
   TargetData* GetData() const { return _data; }

   void SetUnitType(SOC_UNIT_TYPE unit_type);
   SOC_UNIT_TYPE GetUnitType() const;

   // 2026-09-02 R1: 端点插补类型访问器（写入/读取 _data->motion_type）
   void SetMotionType(ACTION_MOTION_TYPE motion_type);
   ACTION_MOTION_TYPE GetMotionType() const;

   void ResetPoseFlag();
   void ResetJointsFlag();

   double GetJointSpeedLimit();
   double GetJointAccLimit();

   void SetName (const char * name);
   void GetName (char * name);

   void SetPose (const math::Pose<> pose);
   void SetPose (const math::Cartesian3<double> position, const math::Quaternion<double> orientation,const TARGET_CONFIG config);
   void SetPose (SE3_MATRIX se3, const TARGET_CONFIG config);
   void GetPose (SE3_MATRIX * se3, TARGET_CONFIG * config);
   void GetPose (math::Cartesian3<double> * position, math::Quaternion<double> * orientation, TARGET_CONFIG * config);
   math::Pose<> GetPose();

   void SetSocJoints (const SOC_JOINTS joints_value);
   virtual void SetJoints (double * joint_value);
   void GetJoints(SOC_JOINTS * joints_value) const;
   JOINT_DATA GetJointData () const;

   int GetJointNumber() const;
   double GetJoint(int joint_no);

   const SOC_JOINTS& GetSocJoints() const;

   double& Joint(int i);
   const double& Joint(int i) const;

   double* JointData();
   const double* JointData() const;

   void SetConfig (const TARGET_CONFIG config_in);
   math::Cartesian3<double> GetPosition() const;
   math::Quaternion<double> GetOrientation() const;

   bool IsJointsValid() const;
   bool IsPoseValid() const;

   void SetPosition(const math::Cartesian3<double> position);
   void SetOrientation(const math::Quaternion<double> orientation);

   void GetArmAngle (float *arm_angle_in);
   void SetArmAngle (const float * arm_angle_in);

   virtual int CalcPose (const FRAME * tool_frame,const bool robot_fixed_object);
   virtual int CalcJointWithConfig (const FRAME * tool_frame,const bool robot_fixed_object);
   virtual int CalcJoint(const FRAME * tool_frame,const bool robot_fixed_object);
   /// 2026-10-10：bool→int（统一码；bool 不能承载 warning 码，false==0 撞哨兵）。
   virtual int CalcVelocityJacobian (float jacobian[6][6]);

};



#endif // __BASE_TARGET_H__