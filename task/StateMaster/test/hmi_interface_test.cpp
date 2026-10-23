/**
 * @file hmi_interface_test.cpp
 * @brief CHmiInterface 单元测试
 */

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "hmi_interface.hpp"
#include "shm_header.hpp"
#include "confbase/robot.hpp"

using state_master_hmi::CHmiInterface;
using state_master_hmi::MapWireMotionType;
using state_master_hmi::StateMsgToEventType;

extern void* g_shm_base;
extern size_t g_shm_size;

#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

class HmiInterfaceTest : public ::testing::Test {
protected:
    void* shm_ptr_ = nullptr;
    static constexpr size_t kShmSize = 100 * 1024 * 1024;

    void SetUp() override {
        shm_ptr_ = mmap(nullptr, kShmSize, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(shm_ptr_, MAP_FAILED);
        ShmHeader* header = static_cast<ShmHeader*>(shm_ptr_);
        header->magic = SHM_MAGIC;
        header->version = 1;
        header->size = kShmSize;
        header->crc = 0;
        header->region_table_offset = 0;
        g_shm_base = shm_ptr_;
        g_shm_size = kShmSize;
    }

    void TearDown() override {
        if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) munmap(shm_ptr_, kShmSize);
        g_shm_base = nullptr;
        g_shm_size = 0;
    }

    void MakeTask(CStateMasterTask& task) {
        task.SetMainTask();
        char name[32] = "state_master";
        ASSERT_TRUE(task.OpenSharedMemory());
        ASSERT_TRUE(task.InitTask(name));
    }
};

// ---- 映射表 ----
TEST(StateMsgMap, AllKnownMessages) {
    EventType t;
    EXPECT_TRUE(StateMsgToEventType("msg_initialize", &t));  EXPECT_EQ(t, EventType::INIT);
    EXPECT_TRUE(StateMsgToEventType("msg_config", &t));      EXPECT_EQ(t, EventType::CONFIG);
    EXPECT_TRUE(StateMsgToEventType("msg_motor_on", &t));    EXPECT_EQ(t, EventType::MOTOR_ON);
    EXPECT_TRUE(StateMsgToEventType("msg_motor_off", &t));   EXPECT_EQ(t, EventType::MOTOR_OFF);
    EXPECT_TRUE(StateMsgToEventType("msg_control_on", &t));  EXPECT_EQ(t, EventType::CONTROL_ON);
    EXPECT_TRUE(StateMsgToEventType("msg_control_off", &t)); EXPECT_EQ(t, EventType::CONTROL_OFF);
    EXPECT_TRUE(StateMsgToEventType("msg_error", &t));       EXPECT_EQ(t, EventType::ERROR);
    EXPECT_TRUE(StateMsgToEventType("msg_reset_ok", &t));    EXPECT_EQ(t, EventType::RESET);
    EXPECT_TRUE(StateMsgToEventType("msg_estop", &t));       EXPECT_EQ(t, EventType::EMERGENCY_STOP);
}

TEST(StateMsgMap, RejectsUnknown) {
    EventType t;
    EXPECT_FALSE(StateMsgToEventType("msg_bogus", &t));
    EXPECT_FALSE(StateMsgToEventType("", &t));
}

// ---- 绑定与事件注入 ----
TEST_F(HmiInterfaceTest, BindTaskRejectsNull) {
    CHmiInterface hmi;
    EXPECT_FALSE(hmi.BindTask(nullptr));
    EXPECT_FALSE(hmi.IsBound());
}

TEST_F(HmiInterfaceTest, SendFsmEventInjectsIntoOwnRing) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));

    ASSERT_TRUE(hmi.SendFsmEvent("msg_motor_on"));
    EVENT_TYPE ev = EMPTY_EVENT;
    task.GetEvent(ev);
    EXPECT_EQ(ev, MOTOR_ON_EVENT);

    EXPECT_FALSE(hmi.SendFsmEvent("msg_bogus"));
}

TEST_F(HmiInterfaceTest, TaskStatusStringFollowsStatus) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));

    task.SetTaskStatus(TASK_STATUS_TYPE::READY);
    EXPECT_EQ(hmi.GetTaskStatusString(), "sta_ready");
    task.SetTaskStatus(TASK_STATUS_TYPE::RUNNING);
    EXPECT_EQ(hmi.GetTaskStatusString(), "sta_motion");
    task.SetTaskStatus(TASK_STATUS_TYPE::ERROR);
    EXPECT_EQ(hmi.GetTaskStatusString(), "sta_error");
    task.SetTaskStatus(TASK_STATUS_TYPE::EMERGENCY_STOP);
    EXPECT_EQ(hmi.GetTaskStatusString(), "sta_estop");
    task.SetTaskStatus(TASK_STATUS_TYPE::SAFE_STOP);
    EXPECT_EQ(hmi.GetTaskStatusString(), "sta_safe_stop");
}

TEST_F(HmiInterfaceTest, SocFeedbacksJsonGroupsByUnit) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));

    Json::Value v = hmi.BuildSocFeedbacksJson();
    ASSERT_TRUE(v.isArray());
    ASSERT_GT(v.size(), 0u);  // humanoid 配置至少有右臂
    EXPECT_TRUE(v[0]["jointAngle"].isMember("J1"));
}

TEST_F(HmiInterfaceTest, StopProgramFlag) {
    CHmiInterface hmi;
    EXPECT_FALSE(hmi.IsStopProgram());
    hmi.SetStopProgram(true);
    EXPECT_TRUE(hmi.IsStopProgram());
    hmi.SetStopProgram(false);
    EXPECT_FALSE(hmi.IsStopProgram());
}

TEST_F(HmiInterfaceTest, BuildManualActionJointsBranch) {
    CHmiInterface hmi;
    double cv[7] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    ActionGroupSharedData grp;
    ASSERT_TRUE(hmi.BuildManualAction(0, 1,   // 线上协议 1 = 关节
                                      false, cv, grp));
    EXPECT_TRUE(grp.soc_enable[0]);
    EXPECT_FALSE(grp.soc_enable[1]);
    const ActionData& a = grp.actions[0];
    EXPECT_EQ(a.motion_type, ACTION_MOTION_TYPE::ACTION_JOINT);
    EXPECT_EQ(a.unit_type, 0);
    EXPECT_TRUE(a.end_target.is_joints_valid);
    EXPECT_FALSE(a.end_target.is_position_valid);
    for (int i = 0; i < 7; i++) {
        EXPECT_DOUBLE_EQ(a.end_target.joints[i], cv[i]);
    }
}

TEST_F(HmiInterfaceTest, BuildManualActionPoseBranch) {
    CHmiInterface hmi;
    // pose 布局：[x,y,z,qx,qy,qz,qw]（同 ref）
    double cv[7] = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
    ActionGroupSharedData grp;
    ASSERT_TRUE(hmi.BuildManualAction(6, 2,   // 线上协议 2 = 直线
                                      true, cv, grp));
    const TargetData& t = grp.actions[6].end_target;
    EXPECT_TRUE(t.is_position_valid);
    EXPECT_TRUE(t.is_orientation_valid);
    EXPECT_DOUBLE_EQ(t.pose.pos.x, 1.0);
    EXPECT_DOUBLE_EQ(t.pose.pos.y, 2.0);
    EXPECT_DOUBLE_EQ(t.pose.pos.z, 3.0);
    EXPECT_DOUBLE_EQ(t.pose.orient.q1, 0.0);  // qx
    EXPECT_DOUBLE_EQ(t.pose.orient.q0, 1.0);  // qw
}

TEST_F(HmiInterfaceTest, BuildManualActionRejectsBadUnit) {
    CHmiInterface hmi;
    double cv[7] = {};
    ActionGroupSharedData grp;
    EXPECT_FALSE(hmi.BuildManualAction(-1, 1, false, cv, grp));
    EXPECT_FALSE(hmi.BuildManualAction(MAX_NO_OF_SOC, 1, false, cv, grp));
}

TEST_F(HmiInterfaceTest, BuildProgramLineMultiSoc) {
    CHmiInterface hmi;
    // 行内两个 SOC：右臂 + 左臂（关节模式）
    Json::Value line(Json::arrayValue);
    Json::Value a0;
    a0["name"] = "r_arm_move";
    a0["unitType"] = 0;
    a0["motionType"] = 1;   // 线上协议 1 = 关节（注释：关节模式）
    a0["is_pose_valid"] = false;
    a0["speed"]["tcp_m_s"] = 0.5;
    a0["speed"]["ori_rad_s"] = 0.1;
    a0["speed"]["joint_rad_s"] = 0.8;
    a0["speed"]["ext_rad_s"] = 0.0;
    for (int i = 0; i < 7; i++) a0["commandValue"][i] = 0.1 * i;
    Json::Value a1 = a0;
    a1["unitType"] = 6;
    line.append(a0);
    line.append(a1);

    ActionGroupSharedData grp;
    std::string err;
    ASSERT_TRUE(hmi.BuildProgramLine(line, 3, grp, &err)) << err;
    EXPECT_TRUE(grp.soc_enable[0]);
    EXPECT_TRUE(grp.soc_enable[6]);
    EXPECT_EQ(grp.action_group_index, 3);
    EXPECT_STREQ(grp.actions[0].name, "r_arm_move");
    EXPECT_DOUBLE_EQ(grp.actions[0].action_speed.tcp_m_s, 0.5);
    EXPECT_DOUBLE_EQ(grp.actions[0].action_speed.joint_rad_s, 0.8);
    EXPECT_DOUBLE_EQ(grp.actions[6].end_target.joints[6], 0.6);
}

TEST_F(HmiInterfaceTest, BuildProgramLineRejectsBadCommandValue) {
    CHmiInterface hmi;
    Json::Value line(Json::arrayValue);
    Json::Value a0;
    a0["unitType"] = 0;
    a0["motionType"] = 1;   // 线上协议 1 = 关节
    a0["is_pose_valid"] = false;
    a0["commandValue"][0] = 0.1;  // 只有 1 个元素，不足 7
    line.append(a0);

    ActionGroupSharedData grp;
    std::string err;
    EXPECT_FALSE(hmi.BuildProgramLine(line, 0, grp, &err));
    EXPECT_FALSE(err.empty());
}

TEST_F(HmiInterfaceTest, SubmitActionsIntoShmQueues) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));

    double cv[7] = {0.1};
    ActionGroupSharedData grp;
    ASSERT_TRUE(hmi.BuildManualAction(0, 2, false, cv, grp));  // wire 2 = 直线
    EXPECT_TRUE(hmi.SubmitManualAction(grp));
    EXPECT_TRUE(hmi.SubmitProgramLine(grp));
    EXPECT_FALSE(task.GetManualActionList()->IsEmpty());
    EXPECT_FALSE(task.GetProgramActionList()->IsEmpty());
}

// 线上协议 motionType（1=关节 2=直线）→ 库枚举映射；非法值拒绝（2026-09-03 起，
// ACTION_NULL 新语义 = 该 SOC 不动，不能再静默放行）
TEST(HmiWireMotionTypeTest, MappingTable) {
    ACTION_MOTION_TYPE mt;
    EXPECT_TRUE(MapWireMotionType(1, &mt));
    EXPECT_EQ(mt, ACTION_MOTION_TYPE::ACTION_JOINT);
    EXPECT_TRUE(MapWireMotionType(2, &mt));
    EXPECT_EQ(mt, ACTION_MOTION_TYPE::ACTION_LINE);
}

TEST(HmiWireMotionTypeTest, InvalidValuesRejected) {
    ACTION_MOTION_TYPE mt;
    EXPECT_FALSE(MapWireMotionType(0, &mt));
    EXPECT_FALSE(MapWireMotionType(3, &mt));
    EXPECT_FALSE(MapWireMotionType(99, &mt));
    EXPECT_FALSE(MapWireMotionType(-1, &mt));
}

TEST_F(HmiInterfaceTest, BuildManualActionRejectsInvalidMotionType) {
    CHmiInterface hmi;
    double cv[7] = {};
    ActionGroupSharedData grp;
    EXPECT_FALSE(hmi.BuildManualAction(0, 0, false, cv, grp));
    EXPECT_FALSE(hmi.BuildManualAction(0, 3, false, cv, grp));
    EXPECT_FALSE(hmi.BuildManualAction(0, 99, false, cv, grp));
}

TEST_F(HmiInterfaceTest, BuildProgramLineRejectsInvalidMotionType) {
    CHmiInterface hmi;
    Json::Value line(Json::arrayValue);
    Json::Value a0;
    a0["name"] = "bad_mt";
    a0["unitType"] = 0;
    a0["motionType"] = 3;   // 非法线上值
    a0["is_pose_valid"] = false;
    a0["speed"]["tcp_m_s"] = 0.5;
    a0["speed"]["joint_rad_s"] = 0.8;
    for (int i = 0; i < 7; i++) a0["commandValue"][i] = 0.1 * i;
    line.append(a0);

    ActionGroupSharedData grp;
    std::string err;
    EXPECT_FALSE(hmi.BuildProgramLine(line, 0, grp, &err));
    EXPECT_NE(err.find("motionType"), std::string::npos);
}

// 端到端：手动动作 motionType=1（线上"关节"）→ ActionData 落 ACTION_JOINT
TEST_F(HmiInterfaceTest, ManualActionMapsWireMotionType) {
    CHmiInterface hmi;
    ActionGroupSharedData out{};
    const double cv[7] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    ASSERT_TRUE(hmi.BuildManualAction(0, 1, false, cv, out));  // wire 1 = 关节
    EXPECT_EQ(out.actions[0].motion_type, ACTION_MOTION_TYPE::ACTION_JOINT);

    ASSERT_TRUE(hmi.BuildManualAction(0, 2, false, cv, out));  // wire 2 = 直线
    EXPECT_EQ(out.actions[0].motion_type, ACTION_MOTION_TYPE::ACTION_LINE);
}

// ---- 模式切换（2026-09-08 手动模式设计）----
TEST(StateMsgMap, ModeStringsMapToMsgType) {
    MsgType mt;
    EXPECT_TRUE(state_master_hmi::StateMsgToMsgType("msg_manual_mode", &mt));
    EXPECT_EQ(mt, MsgType::MANUAL_MODE);
    EXPECT_TRUE(state_master_hmi::StateMsgToMsgType("msg_auto_mode", &mt));
    EXPECT_EQ(mt, MsgType::AUTO_MODE);
    EXPECT_FALSE(state_master_hmi::StateMsgToMsgType("msg_motor_on", &mt));
    EXPECT_FALSE(state_master_hmi::StateMsgToMsgType("msg_bogus", &mt));
}

TEST_F(HmiInterfaceTest, ModeSwitchRejectedWhenPlanningMissing) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));
    std::string err;
    EXPECT_FALSE(hmi.SendModeCommand("msg_auto_mode", &err));  // 未注册 planning_task
    EXPECT_FALSE(err.empty());
}

TEST_F(HmiInterfaceTest, ModeSwitchRejectedWhenPlanningReady) {
    CStateMasterTask task;
    MakeTask(task);
    char name[] = "planning_task";
    ASSERT_TRUE(task.GetStatusManager().NewTaskStatus(name));
    ASSERT_TRUE(task.GetStatusManager().SetStatus(name, READY));
    ASSERT_TRUE(task.GetMsgQueueManager().RegisterQueue(7, "planning_task"));
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));
    std::string err;
    EXPECT_FALSE(hmi.SendModeCommand("msg_manual_mode", &err));
}

TEST_F(HmiInterfaceTest, ModeSwitchAcceptedWhenMotorOff) {
    CStateMasterTask task;
    MakeTask(task);
    char name[] = "planning_task";
    ASSERT_TRUE(task.GetStatusManager().NewTaskStatus(name));
    ASSERT_TRUE(task.GetStatusManager().SetStatus(name, MOTOR_OFF));
    ASSERT_TRUE(task.GetMsgQueueManager().RegisterQueue(7, "planning_task"));
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));
    std::string err;
    ASSERT_TRUE(hmi.SendModeCommand("msg_auto_mode", &err)) << err;
    EXPECT_FALSE(task.IsManualMode());
    // 消息真的进了 planning_task 的消息环
    Message msg;
    ASSERT_TRUE(task.GetMsgQueueManager().Receive(7, msg));
    EXPECT_EQ(msg.type, MsgType::AUTO_MODE);
}

TEST(StateMsgMap, CalibZeroMapsToMsgType) {
    MsgType mt;
    EXPECT_TRUE(state_master_hmi::StateMsgToMsgType("msg_calib_zero", &mt));
    EXPECT_EQ(mt, MsgType::CMD_CALIB_ZERO);
}

TEST_F(HmiInterfaceTest, CalibZeroRejectedWhenFieldbusMissing) {
    CStateMasterTask task;
    MakeTask(task);
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));
    std::string err;
    EXPECT_FALSE(hmi.SendCalibZeroCommand(&err));  // 未注册 fieldbus_task
    EXPECT_FALSE(err.empty());
}

TEST_F(HmiInterfaceTest, CalibZeroAcceptedWhenFieldbusRegistered) {
    CStateMasterTask task;
    MakeTask(task);
    char name[] = "fieldbus_task";
    ASSERT_TRUE(task.GetStatusManager().NewTaskStatus(name));
    ASSERT_TRUE(task.GetMsgQueueManager().RegisterQueue(8, "fieldbus_task"));
    CHmiInterface hmi;
    ASSERT_TRUE(hmi.BindTask(&task));
    std::string err;
    ASSERT_TRUE(hmi.SendCalibZeroCommand(&err)) << err;
    Message msg;
    ASSERT_TRUE(task.GetMsgQueueManager().Receive(8, msg));
    EXPECT_EQ(msg.type, MsgType::CMD_CALIB_ZERO);
}
