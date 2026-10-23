/**
 * @file planning_task_smoke_test.cpp
 * @brief Smoke test for Planning task
 */

#include <gtest/gtest.h>
#include "planning_task.hpp"
#include "task_frame.hpp"
#include "main_fsm.hpp"
#include "tf_event.hpp"
#include "tf_shm_region.hpp"
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

class PlanningTaskSmokeTest : public ::testing::Test {
protected:
    void* _mem = nullptr;

    void SetUp() override {
        _mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_mem, MAP_FAILED);

        ShmHeader* header = static_cast<ShmHeader*>(_mem);
        header->magic = SHM_MAGIC;
        header->version = kShmLayoutVersion;
        header->size = 100 * 1024 * 1024;

        g_shm_base = _mem;
        g_shm_size = 100 * 1024 * 1024;
    }

    void TearDown() override {
        if (_mem != MAP_FAILED && _mem != nullptr) {
            munmap(_mem, 100 * 1024 * 1024);
        }
        g_shm_base = nullptr;
        g_shm_size = 0;
    }
};

TEST_F(PlanningTaskSmokeTest, InitTaskSucceeds) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    EXPECT_TRUE(task.InitTask(name));
}

TEST_F(PlanningTaskSmokeTest, InitialFsmStateIsUninitiated) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        UNINITIATED_STATE<CPlanningTask>>());
}

TEST_F(PlanningTaskSmokeTest, FullFsmTransitionSequence) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.DispatchEvent(INIT_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        UNCONFIGED_STATE<CPlanningTask>>());

    task.DispatchEvent(CONFIG_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        MOTOROFF_STATE<CPlanningTask>>());

    task.DispatchEvent(MOTOR_ON_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        READY_STATE<CPlanningTask>>());

    task.DispatchEvent(CONTROL_ON_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        MOTION_STATE<CPlanningTask>>());

    task.DispatchEvent(CONTROL_OFF_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        READY_STATE<CPlanningTask>>());

    task.DispatchEvent(MOTOR_OFF_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        MOTOROFF_STATE<CPlanningTask>>());
}

TEST_F(PlanningTaskSmokeTest, MainLoopRunsInMotionState) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);
    task.DispatchEvent(CONTROL_ON_EVENT);

    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(task.MainLoop(), 0);
    }
}

TEST_F(PlanningTaskSmokeTest, MainLoopRunsInReadyState) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);

    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(task.MainLoop(), 0);
    }
}

TEST_F(PlanningTaskSmokeTest, ErrorAndResetPath) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);
    task.DispatchEvent(ERROR_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        ERROR_STATE<CPlanningTask>>());

    task.DispatchEvent(RESET_EVENT);
    EXPECT_TRUE(BaseFSM<CPlanningTask>::is_in_state<
        MOTOROFF_STATE<CPlanningTask>>());
}

TEST_F(PlanningTaskSmokeTest, MotionStateRunDoesNotCrash) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);
    task.DispatchEvent(CONTROL_ON_EVENT);

    EXPECT_NO_FATAL_FAILURE(task.MotionStateRun());
}

TEST_F(PlanningTaskSmokeTest, ExitFlagWorks) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    task.SetExitFlag(true);
    EXPECT_TRUE(task.GetExitFlag());
}

// MainLoop 端到端：manual 环中的动作组经 ppwodyn 规划产出轨迹段。
// （2026-09-08 模式设计：消费窗口 READY/MOTION，默认 MANUAL 模式消费 manual 环）
TEST_F(PlanningTaskSmokeTest, MainLoopPlansPushedActionGroup) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    // 推进到 READY（消费窗口为 READY/MOTION）。
    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);

    // 反馈 = 就绪姿态（零位姿 FK 退化不可解）；humanoid 右臂 = 全局前 7 关节。
    FEEDBACK_ALL_JOINTS feedback{};
    const double kReadyPose[7] = {0, -0.5, 0.5, 0, 0.5, 0, 0};
    for (int j = 0; j < 7; ++j) feedback.joints[j].position_feedback = kReadyPose[j];
    task.GetFeedbackManager().UpdateJoints(feedback);

    // 合成单段关节动作组（关节目标，避开 IK 可达性）。
    ActionGroupSharedData group{};
    ActionData& a = group.actions[0];  // HumanoidSocUnitType::RIGHT_ARM
    a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(0);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    for (int j = 0; j < 7; ++j) t.joints[j] = kReadyPose[j] + 0.05 * (j + 1);
    t.joint_number = 7;
    t.is_joints_valid = true;
    group.soc_enable[0] = true;
    ASSERT_TRUE(task.GetManualActionList()->TryPush(group));

    task.MainLoop();

    PathSplineData out{};
    EXPECT_TRUE(task.GetTrajectoryList()->TryPop(out));
    EXPECT_GT(out.path_time, 0.0);
}

// 模式环选择：MANUAL（默认）不消费程序环；切 AUTO（须先离开 READY/MOTION）后消费。
TEST_F(PlanningTaskSmokeTest, ModeSwitchSelectsRing) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));
    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);  // READY

    FEEDBACK_ALL_JOINTS feedback{};
    const double kReadyPose[7] = {0, -0.5, 0.5, 0, 0.5, 0, 0};
    for (int j = 0; j < 7; ++j) feedback.joints[j].position_feedback = kReadyPose[j];
    task.GetFeedbackManager().UpdateJoints(feedback);

    ActionGroupSharedData group{};
    ActionData& a = group.actions[0];
    a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(0);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    for (int j = 0; j < 7; ++j) t.joints[j] = kReadyPose[j] + 0.05 * (j + 1);
    t.joint_number = 7;
    t.is_joints_valid = true;
    group.soc_enable[0] = true;

    // MANUAL（默认）：程序环动作不被消费
    ASSERT_TRUE(task.GetProgramActionList()->TryPush(group));
    task.MainLoop();
    PathSplineData out{};
    EXPECT_FALSE(task.GetTrajectoryList()->TryPop(out));

    // READY 中模式消息被防御性拒绝（消费即弃）：仍不消费程序环
    ASSERT_TRUE(task.GetMsgQueueManager().Send("planning_task_test", MsgType::AUTO_MODE));
    task.MainLoop();
    EXPECT_FALSE(task.GetTrajectoryList()->TryPop(out));

    // 下电后重新发送模式消息（MOTOR_OFF 允许切换），再上电回 READY → 程序环被消费
    task.DispatchEvent(MOTOR_OFF_EVENT);
    ASSERT_TRUE(task.GetMsgQueueManager().Send("planning_task_test", MsgType::AUTO_MODE));
    task.MainLoop();  // MOTOR_OFF 中消费模式消息
    task.DispatchEvent(MOTOR_ON_EVENT);
    task.MainLoop();
    EXPECT_TRUE(task.GetTrajectoryList()->TryPop(out));
    EXPECT_GT(out.path_time, 0.0);
}

// 规划失败路径：不可达位姿 → 自派发 ERROR_EVENT → 状态 ERROR，轨迹环无残留。
TEST_F(PlanningTaskSmokeTest, PlanningFailureSetsErrorStatus) {
    CPlanningTask task;
    task.SetMainTask();
    char name[] = "planning_task_test";
    ASSERT_TRUE(task.InitTask(name));

    // 推进到 READY（ERROR 可从 MOTOROFF/READY/MOTION 进入）。
    task.DispatchEvent(INIT_EVENT);
    task.DispatchEvent(CONFIG_EVENT);
    task.DispatchEvent(MOTOR_ON_EVENT);

    FEEDBACK_ALL_JOINTS feedback{};
    const double kReadyPose[7] = {0, -0.5, 0.5, 0, 0.5, 0, 0};
    for (int j = 0; j < 7; ++j) feedback.joints[j].position_feedback = kReadyPose[j];
    task.GetFeedbackManager().UpdateJoints(feedback);

    ActionGroupSharedData group{};
    ActionData& a = group.actions[0];
    a.motion_type = ACTION_MOTION_TYPE::ACTION_LINE;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(0);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    t.pose.pos.x = 100.0;  // 远超工作空间
    t.pose.pos.y = 100.0;
    t.pose.pos.z = 100.0;
    t.pose.orient.q0 = 1.0;
    t.is_position_valid = true;
    t.is_orientation_valid = true;
    t.is_joints_valid = false;
    group.soc_enable[0] = true;
    ASSERT_TRUE(task.GetManualActionList()->TryPush(group));  // 默认 MANUAL 模式

    task.MainLoop();
    // 第一圈 MainLoop 末尾才自派发 ERROR_EVENT；第二圈 UpdateTaskStatus 同步状态。
    task.MainLoop();

    EXPECT_EQ(task.GetTaskStatus(), TASK_STATUS_TYPE::ERROR);
    PathSplineData out{};
    EXPECT_FALSE(task.GetTrajectoryList()->TryPop(out));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
