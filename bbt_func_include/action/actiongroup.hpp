/**
 * @file actiongroup.hpp
 * @brief
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.2
 * @date 2026-05-09
 *
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-11-13  0.1      Yonglin Chi       first edition
 * 2026-05-09  0.2      Claude            refactor: add Attach/Detach/IsAttached, remove Eigen
 */


#ifndef __ACTION_GROUP_HPP__
#define __ACTION_GROUP_HPP__

#include "action.hpp"
#include "legacy_robot_types.hpp"
#include "targetgroup.hpp"

#include "math/array_operations.h"


typedef struct
{
    double _master_speed = 0.0;
    double _master_acc = 0.0;
    double _master_dec = 0.0;
    double _master_path_length = 0.0;

    double _path_total_time = 0.0;

    double _master_speed_norm = 0.0;
    double _master_acc_norm = 0.0;
    double _master_dec_norm = 0.0;
}JOINT_INTERPOLATION_DATA;


class ACTION_GROUP_CORNER_DATA
{
   public:
      float Lpos1;                  /* in border */
      float Lext1;
      float dL1_comp;
      float fi1;                    /* tool reorientation in corner */
      CARTESIAN n1;
      float Lpos2;                  /* out border */
      float Lext2;
      float dL2_comp;
      float fi2;                    /* tool reorientation in corner */
      CARTESIAN n2;
      /* relative zonesizes before limitation */
      float zpos1;
      float zext1;
      float zpos2;
      float zext2;
      /* Quotas relative zone sizes */
      float Kzori;                  /* min (zori1/zpos1, zori2/zpos2) */
      float Kzrea;                  /* min (zrea1/zpos1, zrea2/zpos2) */
      float Kzlea;                  /* min (zlea1/zpos1, zlea2/zpos2) */
      float Kz;                     /* Min quota of aboves */

      CTargetGroup tg_L1;                /* targets for Lpos1, Lext1 and Lpos2, Lext2 */
      CTargetGroup tg_L2;                /* used when method/frame change in zone */
      CTargetGroup tg_corner;                /* corner point target (not connected) */
      bool internal_finep;         /* sharp corner with zero speed in the middle */

      JointVector joint_L1;       /* joint position for Lpos1, Lext1 */
      JointVector joint_L2;       /* joint position for Lpos2, Lext2 */
      JointVector joint_corner;    /* joint position for corner point */

      JointVector  max_joint_speed_L1; /* max joint speed for Lpos1, Lext1 */
      JointVector  max_joint_speed_L2; /* max joint speed for Lpos2, Lext2 */
      JointVector  max_joint_speed_corner; /* max joint speed for corner point */

      double max_joint_speed;

      float joint_speed_quota;      /* joint_speed segm2 / joint_speed segm1 */
      float joint_distance_quota;   /* joint_distance_2 / joint_distance_1 */
      float cos_alfa_tcp;           /* cos_alfa for the tcp movement */
      float cos_alfa_max;           /* max cos_alfa inclusive reorientation, external axes */
      bool change_of_frames;       /* interpolate corner zone in joints when
                                    * change of coordinated or tool frame */
      bool corner_zone_as_spline;  /* make zones in the same way as for splines_ip */
      bool forced_joint_zone;      /* interpolate corner zone in joints when
                                    * change of frame or change to,form ACTION_JOINT_123 */
      enum action_pos_method posm1; /* used ipol methods in forced_joint_zone */
      enum action_pos_method posm2;
      enum action_ori_method orim1;
      enum action_ori_method orim2;
   };


class CActionGroup
{

private:
    CAction _action_list[MAX_NO_OF_SOC];

    CAction _temp_action;

    ActionGroupSharedData* _shared_data = nullptr;
    CTargetGroup* _target_group = nullptr;  // 绑定目标组视图
    FRAME _tool_frame{};                    // 通用路径工具系

    void GetSocInterpData();


    void GetMasterSpeed(double * master_speed);
    void GetMasterAcc(double * master_acc);
    void GetMasterDec(double * master_dec);
    void GetMasterPathLength(double * master_path_length);

    // 内部辅助
    bool IsBoundNieGroup() const;
    bool SyncGenericPoses();
    bool SyncGenericJoints();
    bool SyncNiePoses();
    bool SyncNieJoints();

public:

    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CActionGroup();
    ~CActionGroup();

    void Attach(ActionGroupSharedData* shared_data);
    void Detach();
    bool IsAttached() const { return _shared_data != nullptr; }
    ActionGroupSharedData* GetSharedData() const { return _shared_data; }

    void JointInterpPrepare();

    ACTION_GROUP_INTERPOLATION_DATA GetActionGroupInterpData(){
        return _shared_data ? _shared_data->action_group_interp_data : ACTION_GROUP_INTERPOLATION_DATA{};
    }
    ACTION_GROUP_SPEED_DATA GetActionGroupSpeed(){
        return _shared_data ? _shared_data->action_group_speed : ACTION_GROUP_SPEED_DATA{};
    }
    void SetActionGroupSpeed(ACTION_GROUP_SPEED_DATA speed_data) {
        if (_shared_data) _shared_data->action_group_speed = speed_data;
    }
    void SetPreActionGroupSpeed(ACTION_GROUP_SPEED_DATA speed_data) {
        if (_shared_data) _shared_data->pre_action_group_speed = speed_data;
    }
    void SetNextActionGroupSpeed(ACTION_GROUP_SPEED_DATA speed_data) {
        if (_shared_data) _shared_data->next_action_group_speed = speed_data;
    }

    // 新增：绑定/解绑目标组视图
    void AttachTargetGroup(CTargetGroup* target_group);
    void DetachTargetGroup();
    bool HasTargetGroup() const { return _target_group != nullptr; }
    CTargetGroup* GetTargetGroup() const { return _target_group; }

    // 新增：显式同步
    bool SyncPoses();   // 根据关节角计算/刷新位姿
    bool SyncJoints();  // 根据位姿计算/刷新关节角
    bool SyncTargets() { return SyncPoses() && SyncJoints(); }

    // 新增：工具系（通用 Humanoid 路径使用；NIE 路径忽略）
    void SetToolFrame(const FRAME& tool_frame);
    const FRAME& GetToolFrame() const;

    bool SetAction(SOC_UNIT_TYPE unit_type, CAction action);
    bool GetAction(SOC_UNIT_TYPE unit_type, CAction ** action);
    bool RemoveAction(SOC_UNIT_TYPE unit_type);

    void SetStartTargets(CTargetGroup & start_targets);
    void SetEndTargets(CTargetGroup & end_targets);

    bool SetData(PROGRAM_LINE_DATA program_line);

    void GetJointPos(double norm_pos, JOINTS_GROUP * joints_group);


    void GetJointInterpData(JOINT_INTERPOLATION_DATA * joint_interp_data);

    bool Execute(int engine_index);

};

#endif //__ACTION_GROUP_HPP__
