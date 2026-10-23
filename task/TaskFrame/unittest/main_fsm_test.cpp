/**
 * @file main_fsm_test.cpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2025-01-16
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2025-01-16  0.1      Yonglin Chi       first edition
 */

#include <gtest/gtest.h>

#include "task_frame.hpp"
#include "main_fsm.hpp"

FSM_INITIAL_STATE(BaseFSM<CTaskFrame>, UNINITIATED_STATE<CTaskFrame>);

using _frame_task_fsm = BaseFSM<CTaskFrame>;

// ---- ESTOP 测试：计数钩子任务，不触碰 shm（无需 InitTask） ----
class EstopCountingTask : public CTaskFrame {
public:
    int estop_count = 0;
    int estop_reset_count = 0;
    int estop_reset_ret = 0;
    char call_seq[16] = {0};
    int call_seq_len = 0;

    int EstopAction() override {
        estop_count++;
        return CTaskFrame::EstopAction();
    }
    int EstopResetAction() override {
        estop_reset_count++;
        return estop_reset_ret;
    }
    int ControlOffAction() override {
        if (call_seq_len < 15) call_seq[call_seq_len++] = 'c';
        return 0;
    }
    int MotorOffAction() override {
        if (call_seq_len < 15) call_seq[call_seq_len++] = 'm';
        return 0;
    }
};

FSM_INITIAL_STATE(BaseFSM<EstopCountingTask>, UNINITIATED_STATE<EstopCountingTask>);
using _estop_fsm = BaseFSM<EstopCountingTask>;

class EstopUninitTask : public CTaskFrame {
public:
    int estop_reset_ret = 0;
    int EstopResetAction() override { return estop_reset_ret; }
};

FSM_INITIAL_STATE(BaseFSM<EstopUninitTask>, UNINITIATED_STATE<EstopUninitTask>);
using _estop_uninit_fsm = BaseFSM<EstopUninitTask>;

// ---- 规划停测试：钩子任务，记录调用序列，不触碰 shm ----
class PlannedStopTask : public CTaskFrame {
public:
    bool start_ret = true;
    int start_count = 0;
    int run_count = 0;
    int run_done_at = 2;          // 第 N 次 run 才返回 true（停完）
    int error_action_count = 0;
    int estop_action_count = 0;

    bool StartEmergencyPlannedStop() override { start_count++; return start_ret; }
    bool EmergencyPlannedStopRun() override { run_count++; return run_count >= run_done_at; }
    int ErrorAction() override { error_action_count++; return 0; }
    int EstopAction() override { estop_action_count++; return CTaskFrame::EstopAction(); }
};

FSM_INITIAL_STATE(BaseFSM<PlannedStopTask>, UNINITIATED_STATE<PlannedStopTask>);
using _stop_fsm = BaseFSM<PlannedStopTask>;

// MOTION + ERROR：需要规划停 → 中间态 → 停完才进 ERROR
TEST(MAINFSM, MotionErrorWithPlannedStop)
{
    PlannedStopTask task;
    _stop_fsm::setTask(&task);
    _stop_fsm::start();

    FSM_Init_Event init_event; FSM_Config_Event config_event;
    FSM_MotorOn_Event motor_on_event; FSM_ControlOn_Event control_on_event;
    _stop_fsm::dispatch(init_event);
    _stop_fsm::dispatch(config_event);
    _stop_fsm::dispatch(motor_on_event);
    _stop_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STATE<PlannedStopTask>>());

    FSM_Error_Event error_event;
    _stop_fsm::dispatch(error_event);
    EXPECT_EQ(task.start_count, 1);
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STOPPING_STATE<PlannedStopTask>>());
    EXPECT_EQ(task.error_action_count, 0);   // ErrorAction 必须等停完

    // 停机期间 ERROR/ESTOP 事件被忽略（停完再进）
    _stop_fsm::dispatch(error_event);
    FSM_Emergency_Stop_Event estop_event;
    _stop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STOPPING_STATE<PlannedStopTask>>());
    EXPECT_EQ(task.estop_action_count, 0);

    // 第 1 次 run：未停完，留在中间态
    _stop_fsm::current_state_ptr->run();
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STOPPING_STATE<PlannedStopTask>>());
    // 第 2 次 run：停完 → ErrorAction → ERROR
    _stop_fsm::current_state_ptr->run();
    ASSERT_TRUE(_stop_fsm::is_in_state<ERROR_STATE<PlannedStopTask>>());
    EXPECT_EQ(task.error_action_count, 1);
    EXPECT_EQ(task.estop_action_count, 0);   // 期间的 estop 被忽略，未误触发
}

// MOTION + ESTOP：需要规划停 → 中间态 → 停完才进 ESTOP
TEST(MAINFSM, MotionEstopWithPlannedStop)
{
    PlannedStopTask task;
    _stop_fsm::setTask(&task);
    _stop_fsm::start();

    FSM_Init_Event init_event; FSM_Config_Event config_event;
    FSM_MotorOn_Event motor_on_event; FSM_ControlOn_Event control_on_event;
    _stop_fsm::dispatch(init_event);
    _stop_fsm::dispatch(config_event);
    _stop_fsm::dispatch(motor_on_event);
    _stop_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STATE<PlannedStopTask>>());

    FSM_Emergency_Stop_Event estop_event;
    _stop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_stop_fsm::is_in_state<MOTION_STOPPING_STATE<PlannedStopTask>>());
    EXPECT_EQ(task.estop_action_count, 0);   // EstopAction 必须等停完

    task.run_done_at = 1;                    // 一次 run 即停完
    _stop_fsm::current_state_ptr->run();
    ASSERT_TRUE(_stop_fsm::is_in_state<ESTOP_STATE<PlannedStopTask>>());
    EXPECT_EQ(task.estop_action_count, 1);
}

// 回归锁：默认钩子（无需停）→ MOTION 直达目标态，行为与既有版本一致
TEST(MAINFSM, MotionErrorEstopDirectWithoutPlannedStop)
{
    EstopCountingTask task;   // 不重写停机钩子，走基类默认 false
    _estop_fsm::setTask(&task);
    _estop_fsm::start();

    FSM_Init_Event init_event; FSM_Config_Event config_event;
    FSM_MotorOn_Event motor_on_event; FSM_ControlOn_Event control_on_event;
    _estop_fsm::dispatch(init_event);
    _estop_fsm::dispatch(config_event);
    _estop_fsm::dispatch(motor_on_event);
    _estop_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTION_STATE<EstopCountingTask>>());

    FSM_Error_Event error_event;
    _estop_fsm::dispatch(error_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ERROR_STATE<EstopCountingTask>>());

    // 复位后回 MOTION 再验证 ESTOP 直达
    FSM_Reset_Event reset_event;
    _estop_fsm::dispatch(reset_event);
    _estop_fsm::dispatch(motor_on_event);
    _estop_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTION_STATE<EstopCountingTask>>());
    FSM_Emergency_Stop_Event estop_event;
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
}

// 从 UNCONFIGED / MOTOROFF / READY / MOTION / ERROR 五个起点急停 + 幂等 + 复位链
TEST(MAINFSM, EstopWalkThroughStates)
{
    EstopCountingTask task;
    _estop_fsm::setTask(&task);
    _estop_fsm::start();
    ASSERT_TRUE(_estop_fsm::is_in_state<UNINITIATED_STATE<EstopCountingTask>>());

    FSM_Emergency_Stop_Event estop_event;
    FSM_Reset_Event reset_event;

    // --- 起点 UNCONFIGED ---
    FSM_Init_Event init_event;
    _estop_fsm::dispatch(init_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<UNCONFIGED_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    EXPECT_EQ(task.estop_count, 1);
    // 幂等：ESTOP 中重复急停不再动作
    _estop_fsm::dispatch(estop_event);
    EXPECT_EQ(task.estop_count, 1);
    // ESTOP 中其他事件被忽略
    FSM_MotorOn_Event motor_on_event;
    _estop_fsm::dispatch(motor_on_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    // 复位 → MOTOROFF
    _estop_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTOROFF_STATE<EstopCountingTask>>());
    EXPECT_EQ(task.estop_reset_count, 1);
    // 默认 EstopAction 链：先 ControlOff 后 MotorOff
    ASSERT_GE(task.call_seq_len, 2);
    EXPECT_EQ(task.call_seq[0], 'c');
    EXPECT_EQ(task.call_seq[1], 'm');

    // --- 起点 MOTOROFF ---
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTOROFF_STATE<EstopCountingTask>>());

    // --- 起点 READY ---
    _estop_fsm::dispatch(motor_on_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<READY_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTOROFF_STATE<EstopCountingTask>>());

    // --- 起点 MOTION ---
    FSM_ControlOn_Event control_on_event;
    _estop_fsm::dispatch(motor_on_event);
    _estop_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTION_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTOROFF_STATE<EstopCountingTask>>());

    // --- 起点 ERROR（急停优先级高于错误锁存） ---
    FSM_Error_Event error_event;
    _estop_fsm::dispatch(motor_on_event);
    _estop_fsm::dispatch(control_on_event);
    _estop_fsm::dispatch(error_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ERROR_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<ESTOP_STATE<EstopCountingTask>>());
    _estop_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_fsm::is_in_state<MOTOROFF_STATE<EstopCountingTask>>());

    EXPECT_EQ(task.estop_count, 5);        // 五个起点各一次（ESTOP 中重复那次不计）
    EXPECT_EQ(task.estop_reset_count, 5);
}

// 起点 UNINITIATED 急停 + 复位失败留在 ESTOP
TEST(MAINFSM, EstopFromUninitiatedAndResetFailure)
{
    EstopUninitTask task;
    _estop_uninit_fsm::setTask(&task);
    _estop_uninit_fsm::start();
    ASSERT_TRUE(_estop_uninit_fsm::is_in_state<UNINITIATED_STATE<EstopUninitTask>>());

    FSM_Emergency_Stop_Event estop_event;
    _estop_uninit_fsm::dispatch(estop_event);
    ASSERT_TRUE(_estop_uninit_fsm::is_in_state<ESTOP_STATE<EstopUninitTask>>());

    // 复位动作返回非 0 → 留在 ESTOP
    task.estop_reset_ret = 1;
    FSM_Reset_Event reset_event;
    _estop_uninit_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_uninit_fsm::is_in_state<ESTOP_STATE<EstopUninitTask>>());

    // 复位成功 → MOTOROFF
    task.estop_reset_ret = 0;
    _estop_uninit_fsm::dispatch(reset_event);
    ASSERT_TRUE(_estop_uninit_fsm::is_in_state<MOTOROFF_STATE<EstopUninitTask>>());
}

TEST(MAINFSM, StateTest)
{
    // TODO: add test code here

    char task_name[32] = "testtask";

    CTaskFrame task_frame;
    task_frame.SetMainTask();
    task_frame.InitTask(task_name);

    _frame_task_fsm::setTask(&task_frame);
    _frame_task_fsm::start();
    ASSERT_TRUE(_frame_task_fsm::is_in_state<UNINITIATED_STATE<CTaskFrame>>());
    
    FSM_Init_Event init_event;
    _frame_task_fsm::dispatch(init_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<UNCONFIGED_STATE<CTaskFrame>>());

    FSM_Config_Event config_event;
    _frame_task_fsm::dispatch(config_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<MOTOROFF_STATE<CTaskFrame>>());

    FSM_MotorOn_Event motor_on_event;
    _frame_task_fsm::dispatch(motor_on_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<READY_STATE<CTaskFrame>>());

    FSM_ControlOn_Event control_on_event;
    _frame_task_fsm::dispatch(control_on_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<MOTION_STATE<CTaskFrame>>());

    FSM_ControlOff_Event control_off_event;
    _frame_task_fsm::dispatch(control_off_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<READY_STATE<CTaskFrame>>());

    FSM_MotorOff_Event motor_off_event;
    _frame_task_fsm::dispatch(motor_off_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<MOTOROFF_STATE<CTaskFrame>>());

    FSM_Error_Event error_event;
    _frame_task_fsm::dispatch(error_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<ERROR_STATE<CTaskFrame>>());

    FSM_Reset_Event reset_event;
    _frame_task_fsm::dispatch(reset_event);
    ASSERT_TRUE(_frame_task_fsm::is_in_state<MOTOROFF_STATE<CTaskFrame>>());


}

int main(int argc, char *argv[])
{
    
	testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();

    // return 0;
}