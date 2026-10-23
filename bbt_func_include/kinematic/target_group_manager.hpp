/**
 * @file target_group_manager.hpp
 * @brief TargetGroupSharedData pool manager using GlobalData template.
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-08
 *
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 */

#ifndef _TARGET_GROUP_MANAGER_H_
#define _TARGET_GROUP_MANAGER_H_

#include "global_data.hpp"
#include "basetarget.hpp"

#define MAX_TARGETGROUP_NUM 500

class TargetGroupManager {
public:
    TargetGroupManager();
    ~TargetGroupManager();

    bool Create(const char* name, int key);
    bool Connect(const char* name, int key);
    void Disconnect();
    void Destroy();

    TargetGroupSharedData* Acquire();
    void Retain(TargetGroupSharedData* tg);
    bool Release(TargetGroupSharedData* tg);

    int UsedCount() const;
    int AvailableCount() const;
    int Capacity() const;
    int GetUseCount(const TargetGroupSharedData* tg) const;

private:
    GlobalData<TargetGroupSharedData, MAX_TARGETGROUP_NUM> pool_;
};

#endif // _TARGET_GROUP_MANAGER_H_
