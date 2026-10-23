#include <gtest/gtest.h>
#include "state_master.hpp"
#include "task_frame.hpp"
#include "shm_header.hpp"
#include <sys/mman.h>

// Externs from TaskFrame
extern void* g_shm_base;
extern size_t g_shm_size;

// Required when TASK_FRAME_TEST is defined (task_frame.cpp references this)
#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

class StateMasterTest : public ::testing::Test {
protected:
    void* _shm_ptr = nullptr;
    static constexpr size_t SHM_SIZE = 100 * 1024 * 1024;

    void SetUp() override {
        // Allocate anonymous shared memory for testing
        _shm_ptr = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_shm_ptr, MAP_FAILED);
        ASSERT_NE(_shm_ptr, nullptr);

        // Write ShmHeader
        ShmHeader* header = static_cast<ShmHeader*>(_shm_ptr);
        header->magic = SHM_MAGIC;
        header->version = 1;
        header->size = SHM_SIZE;
        header->crc = 0;
        header->region_table_offset = 0;

        // Set globals so TaskFrame can use them
        g_shm_base = _shm_ptr;
        g_shm_size = SHM_SIZE;
    }

    void TearDown() override {
        if (_shm_ptr != nullptr && _shm_ptr != MAP_FAILED) {
            munmap(_shm_ptr, SHM_SIZE);
        }
        g_shm_base = nullptr;
        g_shm_size = 0;
    }
};

TEST_F(StateMasterTest, InitTaskCreatesFsmInUninitiatedState)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    EXPECT_EQ(task.GetFSMState(), UNINTIATED);
}

TEST_F(StateMasterTest, DispatchInitEventTransitionsToUnconfiged)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    task.DispatchEvent(INIT_EVENT);
    EXPECT_EQ(task.GetFSMState(), UNCONFIGED);
}

TEST_F(StateMasterTest, FullFsmTransitionSequence)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    // Initial state
    EXPECT_EQ(task.GetFSMState(), UNINTIATED);

    // UNINITIATED -> Init -> UNCONFIGED
    task.DispatchEvent(INIT_EVENT);
    EXPECT_EQ(task.GetFSMState(), UNCONFIGED);

    // UNCONFIGED -> Config -> MOTOR_OFF
    task.DispatchEvent(CONFIG_EVENT);
    EXPECT_EQ(task.GetFSMState(), MOTOR_OFF);

    // MOTOR_OFF -> MotorOn -> READY
    task.DispatchEvent(MOTOR_ON_EVENT);
    EXPECT_EQ(task.GetFSMState(), READY);

    // READY -> ControlOn -> MOTION (RUNNING)
    task.DispatchEvent(CONTROL_ON_EVENT);
    EXPECT_EQ(task.GetFSMState(), RUNNING);

    // MOTION -> ControlOff -> READY
    task.DispatchEvent(CONTROL_OFF_EVENT);
    EXPECT_EQ(task.GetFSMState(), READY);

    // READY -> MotorOff -> MOTOR_OFF
    task.DispatchEvent(MOTOR_OFF_EVENT);
    EXPECT_EQ(task.GetFSMState(), MOTOR_OFF);
}

TEST_F(StateMasterTest, BroadcastEventsViaNewSubsystem)
{
    // Create main task
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    // Create a sub-task that will receive broadcast events
    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // Transition main task to MOTOR_OFF state so MotorOnAction will be triggered
    main_task.DispatchEvent(INIT_EVENT);
    EXPECT_EQ(main_task.GetFSMState(), UNCONFIGED);
    main_task.DispatchEvent(CONFIG_EVENT);
    EXPECT_EQ(main_task.GetFSMState(), MOTOR_OFF);

    // Drain all stale events from sub-task (task_id 1) that accumulated during Init/Config
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}

    // Trigger MotorOn on main task which broadcasts MOTOR_ON event
    main_task.DispatchEvent(MOTOR_ON_EVENT);
    EXPECT_EQ(main_task.GetFSMState(), READY);

    // Verify sub-task received the MOTOR_ON event via new event subsystem
    // The sub-task is task_id 1 (state_master is 0)
    Event ev;
    bool has_event = main_task.GetEventManager().HasEvent(1);
    EXPECT_TRUE(has_event);
    EXPECT_TRUE(main_task.GetEventManager().ConsumeEvent(1, ev));
    EXPECT_EQ(ev.type, EventType::MOTOR_ON);
}

TEST_F(StateMasterTest, ErrorEventTransitionsToErrorState)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    // Init -> UNCONFIGED
    task.DispatchEvent(INIT_EVENT);
    EXPECT_EQ(task.GetFSMState(), UNCONFIGED);

    // Config -> MOTOR_OFF
    task.DispatchEvent(CONFIG_EVENT);
    EXPECT_EQ(task.GetFSMState(), MOTOR_OFF);

    // Error -> ERROR
    task.DispatchEvent(ERROR_EVENT);
    EXPECT_EQ(task.GetFSMState(), ERROR);
}

TEST_F(StateMasterTest, ResetFromErrorGoesToMotorOff)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    // Init -> UNCONFIGED
    task.DispatchEvent(INIT_EVENT);
    EXPECT_EQ(task.GetFSMState(), UNCONFIGED);

    // Config -> MOTOR_OFF
    task.DispatchEvent(CONFIG_EVENT);
    EXPECT_EQ(task.GetFSMState(), MOTOR_OFF);

    // Error -> ERROR
    task.DispatchEvent(ERROR_EVENT);
    EXPECT_EQ(task.GetFSMState(), ERROR);

    // Reset -> MOTOR_OFF
    task.DispatchEvent(RESET_EVENT);
    EXPECT_EQ(task.GetFSMState(), MOTOR_OFF);
}

TEST_F(StateMasterTest, CombinedBitmaskInitConfig)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    // 优先级序：CONFIG 先派发（UNINITIATED 不 react，忽略），INIT 后派发生效
    task.DispatchEvent(static_cast<EVENT_TYPE>(INIT_EVENT | CONFIG_EVENT));
    EXPECT_EQ(task.GetFSMState(), UNCONFIGED);
}

TEST_F(StateMasterTest, CombinedBitmaskErrorBeatsMotorOn)
{
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    ASSERT_EQ(task.GetFSMState(), MOTOR_OFF);

    // 优先级序：ERROR 先派发进 ERROR_STATE，MOTOR_ON 在 ERROR_STATE 无 react 被忽略
    task.DispatchEvent(static_cast<EVENT_TYPE>(MOTOR_ON_EVENT | ERROR_EVENT));
    EXPECT_EQ(task.GetFSMState(), ERROR);
}

TEST_F(StateMasterTest, SyncWaitLimitConfigurable)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    // 子任务（task_id 1）状态停在 UNINTIATED，与期望的 UNCONFIGED 不匹配
    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    EXPECT_EQ(main_task.GetSyncWaitLimit(), 80);  // 默认值保持现状
    main_task.SetSyncWaitLimit(3);
    EXPECT_EQ(main_task.GetSyncWaitLimit(), 3);

    // InitAction 置 _expected_status=UNCONFIGED、_other_task_syn_wait=1
    main_task.DispatchEvent(INIT_EVENT);

    // 排掉 INIT 广播，便于后面只检测 ERROR 广播
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}

    // 每次不匹配 +1；超过 3 后（第 4 次调用）应触发 ERROR 广播
    for (int i = 0; i < 4; ++i) {
        main_task.UpdateTaskStatus();
    }

    Event ev;
    ASSERT_TRUE(main_task.GetEventManager().ConsumeEvent(1, ev));
    EXPECT_EQ(ev.type, EventType::ERROR);
}

TEST_F(StateMasterTest, EstopDoesNotWaitForSubTasks)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    // 子任务停在 UNINTIATED（模拟 fieldbus 规划停期间合法停留非目标态）
    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // ESTOP：主任务自身立即上报 EMERGENCY_STOP
    main_task.DispatchEvent(EMERGENCY_STOP_EVENT);

    // 排掉 EMERGENCY_STOP 广播，便于后面只检测超时 ERROR 广播
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}

    // 不等待子任务 → UpdateTaskStatus 早退，永不触发超时 ERROR 广播
    for (int i = 0; i < 100; ++i) {   // 覆盖默认窗口 80 周期
        main_task.UpdateTaskStatus();
    }
    Event ev;
    EXPECT_FALSE(main_task.GetEventManager().ConsumeEvent(1, ev));
}

TEST_F(StateMasterTest, SubTaskErrorDetectedImmediately)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // 进入等待同步状态（期望 UNCONFIGED）
    main_task.DispatchEvent(INIT_EVENT);

    // 排掉 INIT 广播
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}

    // 子任务状态置 ERROR，单次 UpdateTaskStatus 即应进 ERROR 态（不等超时）
    sub_task.SetTaskStatus(ERROR);
    main_task.UpdateTaskStatus();

    EXPECT_EQ(main_task.GetFSMState(), ERROR);
    Event ev;
    ASSERT_TRUE(main_task.GetEventManager().ConsumeEvent(1, ev));
    EXPECT_EQ(ev.type, EventType::ERROR);
}

TEST_F(StateMasterTest, ControlOffFromMotionReturnsToReady)
{
    // 回归锁定：CONTROL_OFF 后状态必须正确离开 RUNNING（bug: control-off-status-stuck）
    CStateMasterTask task;
    task.SetMainTask();

    char task_name[32] = "state_master";
    ASSERT_TRUE(task.OpenSharedMemory());
    ASSERT_TRUE(task.InitTask(task_name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);
    task.DispatchEvent(CONTROL_ON_EVENT);
    ASSERT_EQ(task.GetFSMState(), RUNNING);

    task.DispatchEvent(CONTROL_OFF_EVENT);
    EXPECT_EQ(task.GetFSMState(), READY);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// 稳态上抛（2026-09-10 设计）：无同步等待窗口（未发起任何切换）时，
// 子任务 ERROR 也必须被检测并上抛——现行实现漏掉此场景。
TEST_F(StateMasterTest, SubTaskErrorDetectedInSteadyState)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // 不发起任何状态切换（_other_task_syn_wait == 0，稳态）
    sub_task.SetTaskStatus(ERROR);
    main_task.UpdateTaskStatus();

    EXPECT_EQ(main_task.GetFSMState(), ERROR);
    Event ev;
    ASSERT_TRUE(main_task.GetEventManager().ConsumeEvent(1, ev));
    EXPECT_EQ(ev.type, EventType::ERROR);
}

// 边沿语义：同一轮 ERROR 只处置一次；子任务恢复后再 ERROR 可再次上抛
TEST_F(StateMasterTest, SubTaskErrorEdgeDetectionSemantics)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // 第一轮：上抛并进 ERROR
    sub_task.SetTaskStatus(ERROR);
    main_task.UpdateTaskStatus();
    ASSERT_EQ(main_task.GetFSMState(), ERROR);

    // 同一轮重复扫描：不再重复 dispatch（事件环中不应再有第二条 ERROR）
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}
    main_task.UpdateTaskStatus();
    EXPECT_FALSE(main_task.GetEventManager().ConsumeEvent(1, drain));

    // 子任务恢复 → 主任务复位出 ERROR；掩码在主任务非 ERROR 且子任务非 ERROR
    // 的扫描点复位（re-arm）
    sub_task.SetTaskStatus(UNINTIATED);
    main_task.DispatchEvent(RESET_EVENT);  // ERROR → MOTOROFF
    ASSERT_EQ(main_task.GetFSMState(), MOTOR_OFF);
    main_task.UpdateTaskStatus();          // re-arm：双方均非 ERROR 时清掩码位

    sub_task.SetTaskStatus(ERROR);
    main_task.UpdateTaskStatus();
    EXPECT_EQ(main_task.GetFSMState(), ERROR);
}

// RESET 恢复窗口不重复上抛（2026-09-10 multi_process Reset_FromError 实测定律）：
// ERROR 进入窗口中子任务状态滞后（尚未写上 ERROR）不得解除掩码预置
TEST_F(StateMasterTest, NoReescalationDuringResetRecovery)
{
    CStateMasterTask main_task;
    main_task.SetMainTask();
    char main_name[32] = "state_master";
    ASSERT_TRUE(main_task.OpenSharedMemory());
    ASSERT_TRUE(main_task.InitTask(main_name));

    CTaskFrame sub_task;
    char sub_name[32] = "sub_task";
    ASSERT_TRUE(sub_task.OpenSharedMemory());
    ASSERT_TRUE(sub_task.InitTask(sub_name));

    // 主任务主动进 ERROR（ErrorAction 预置全掩码）
    main_task.DispatchEvent(ERROR_EVENT);
    ASSERT_EQ(main_task.GetFSMState(), ERROR);
    // 进入窗口扫描：子任务尚未写上 ERROR（在途延迟）——不得清除掩码位
    main_task.UpdateTaskStatus();
    // 子任务滞后写上 ERROR（广播在途）
    sub_task.SetTaskStatus(ERROR);
    // 主任务 RESET → MOTOROFF，广播 RESET；子任务尚未消费（仍 ERROR）
    main_task.DispatchEvent(RESET_EVENT);
    ASSERT_EQ(main_task.GetFSMState(), MOTOR_OFF);
    Event drain;
    while (main_task.GetEventManager().ConsumeEvent(1, drain)) {}
    // 恢复窗口扫描：掩码若被清除则重复上抛（bug）；正确行为 = 不动
    main_task.UpdateTaskStatus();
    EXPECT_EQ(main_task.GetFSMState(), MOTOR_OFF);
    EXPECT_FALSE(main_task.GetEventManager().ConsumeEvent(1, drain));
}
