/**
 * @file jointaction.hpp
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
 * 2026-05-09  0.2      Claude            add ActionData* constructor
 */

#ifndef __JOINT_ACTION_HPP__
#define __JOINT_ACTION_HPP__

#include "actionbase.hpp"


class CJointAction : public CActionBase
{
public:

    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CJointAction();
    explicit CJointAction(ActionData* data);
    ~CJointAction();


    void GetMasterSpeed(double * master_speed) override;

    void GetMasterAcc(double *  master_acc) override;

    void GetMasterDec(double * master_dec) override;

    void GetMasterPathLength(double * master_path_length) override;

    double CalculateDistance() override;


};

#endif //__JOINT_ACTION_HPP__
