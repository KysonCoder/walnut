/**
 * @file lineaction.hpp
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


#ifndef __LINEACTION_HPP__
#define __LINEACTION_HPP__

#include "actionbase.hpp"

class CLineAction : public CActionBase
{
private:
    /* data */
public:

    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CLineAction ();
    explicit CLineAction(ActionData* data);
    ~CLineAction();

    void GetMasterSpeed(double * master_speed) override;

    void GetMasterAcc(double *  master_acc) override;

    void GetMasterDec(double * master_dec) override;

    void GetMasterPathLength(double * master_path_length) override;

    double CalculateDistance() override;
};



#endif // __LINEACTION_HPP__
