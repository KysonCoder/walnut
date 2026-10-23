/**
 * @file action.hpp
 * @brief
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.2
 * @date 2026-05-09
 *
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-08-23  0.1      Yonglin Chi       first edition
 * 2026-05-09  0.2      Claude            refactor: add Attach/Detach/IsAttached, update includes
 */



#ifndef __ROBOT_ACTION_H__
#define __ROBOT_ACTION_H__

#include <memory>
#include <vector>

#include "legacy_robot_types.hpp"
#include "trajectory_types.hpp"
#include "path_geometry.hpp"
#include "lineaction.hpp"
#include "forceaction.hpp"
#include "jointaction.hpp"
#include "circleaction.hpp"




// typedef struct _action_step_type
// {
// ACTION_MOTION_TYPE actions_type[MAX_NO_OF_SOC];

// _action_step_type & operator = (const _action_step_type& other) {
//         if (this != &other) { // 避免自赋值
//             for(int i=0; i<MAX_NO_OF_SOC; i++)
//                 this->actions_type[i] = other.actions_type[i];
//         }
//         return * this;
//     }

// }ACTION_STEP_TYPE;






class CAction
{
private:
    /* data */

    ACTION_MOTION_TYPE _action_type;
    CActionBase * _active_action;

    CLineAction _line_action;
    CForceAction _force_action;
    CJointAction _joint_action;
    CCircleAction _circle_action;

    CTarget _null_target;




public:

    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CAction();
    // CAction(ACTION_MOTION_TYPE action_type);
    CAction(const CAction & other);
    ~CAction();

    void Attach(ACTION_MOTION_TYPE motion_type, ActionData* data);
    void Detach();
    bool IsAttached() const;

    ACTION_INTERPOLATION_DATA GetInterpolationData();

    void JointInterpPrepare();


    virtual CAction & operator = (const CAction & other);


    void SetName(const char * soc_name);



    void SetEndTarget(CTarget end_target);
    void SetStartTarget(CTarget start_target);
    void SetTargets(CTarget start_target, CTarget end_target);
    void SetSpeed(SPEED_DATA speed);

    ACTION_SPEED_DATA GetActionSpeed() { return _active_action->GetSpeed(); }

    CTarget GetEndTarget();
    CTarget GetStartTarget();

    void SetLineAction(CLineAction line_action);
    void SetForceAction(CForceAction force_action);
    void SetJointAction(CJointAction joint_action);
    void SetCircleAction(CCircleAction circle_action);

    CLineAction & GetLineAction(){return _line_action;}
    CForceAction & GetForceAction(){return _force_action;}
    CJointAction & GetJointAction(){return _joint_action;}
    CCircleAction & GetCircleAction(){return _circle_action;}


    void SetActionType(ACTION_MOTION_TYPE action_type);
    ACTION_MOTION_TYPE GetActionType(){return _action_type;}


    void UpdateAction(CAction action);
    void Clear();


    bool IsValidAction(SOC_UNIT_TYPE unit_type);


    void GetMasterSpeed(double * master_speed);

    void GetMasterAcc(double *  master_acc);

    void GetMasterDec(double *  master_dec);

    void GetMasterPathLength(double * master_path_length);

    void GetJointPos(double norm_pos, SOC_JOINTS * joints);

    /**
     * @brief 插值计算目标点 (delegates to CActionBase::Interpolate)
     */
    void Interpolate(CTarget* p, CTarget* p_start, CTarget* p_end,
                     double L, action_ipol_method method);


};







#endif
