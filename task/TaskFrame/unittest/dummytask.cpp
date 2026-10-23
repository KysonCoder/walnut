/**
 * @file dummytask.cpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2025-01-23
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2025-01-23  0.1      Yonglin Chi       first edition
 */

#include <unistd.h>

#include <cstdio>

#include "task_frame.hpp"
#include "confbase/robot.hpp"

bool DummyTaskInit();

#ifdef TASK_FRAME_TEST
extern TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

extern CTaskFrame new_task;

// extern CRingBuffer<CAction, 32> program_action_list;
// extern CRingBuffer<CActionGroup, 8> manual_action_list;

__attribute__((force_align_arg_pointer))
void SetProgramList(GlobalData * global_data)
{
    // 历史上的 TARGET_GROUP_DATA/CTargetGroup 占位填充已移除
    // （taskframe 的 targetgroup.hpp 占位已转发到 func/kinematic 真实类型）。
    // 本测试的目的只是向 program_list 推入一个动作组元素。
    CActionGroup actiongroup1;

    auto* program_list = new_task.GetProgramActionList();
    if (program_list != nullptr) {
        program_list->TryPush(actiongroup1.GetSharedData());
    }

    // int mem_cmp_result = memcmp(global_data._program_actionlist, &program_action_list, sizeof(CRingBuffer<CAction, 32>));
    // Log_D("mem_cmp_result is %d\n", mem_cmp_result);

    // CAction action1;

    // global_data._program_actionlist->Init();
    // global_data._program_actionlist->SetData(action1);
    
    // CRingBuffer<CAction, 32> * temp_list = & program_action_list;
    // temp_list->SetData(action1);


}

int dummy_main()
{

    printf("Test Task starts!\n");

    if (!DummyTaskInit()) {
        printf("DummyTaskInit failed, skipping SetProgramList\n");
        return 0;
    }

    GlobalData * global_data = new_task.GetGlobalData();

    SetProgramList(global_data);

    {
        TfEventManager& event_mgr = new_task.GetEventManager();
        event_mgr.PublishBroadcast(EventType::MOTOR_OFF);
    }

    char task_name[32] = "dummytask";

    {
        CStatusManager status_mgr(global_data->_task_status.get());
        status_mgr.SetStatus(task_name, MOTOR_OFF);
    }
   


    return 0;

}