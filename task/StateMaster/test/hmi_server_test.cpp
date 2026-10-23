/**
 * @file hmi_server_test.cpp
 * @brief HmiServer HTTP 路由测试：真实 httplib::Client 往返
 */

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "hmi_server.hpp"
#include "shm_header.hpp"

using state_master_hmi::CHmiInterface;
using state_master_hmi::HmiServer;

extern void* g_shm_base;
extern size_t g_shm_size;

#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

class HmiServerTest : public ::testing::Test {
protected:
    void* shm_ptr_ = nullptr;
    static constexpr size_t kShmSize = 100 * 1024 * 1024;
    static constexpr int kPort = 18099;

    CStateMasterTask task_;
    CHmiInterface hmi_;
    HmiServer* server_ = nullptr;

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

        task_.SetMainTask();
        char name[32] = "state_master";
        ASSERT_TRUE(task_.OpenSharedMemory());
        ASSERT_TRUE(task_.InitTask(name));
        ASSERT_TRUE(hmi_.BindTask(&task_));

        server_ = new HmiServer(&hmi_, kPort);
        ASSERT_TRUE(server_->Start());
    }

    void TearDown() override {
        delete server_;
        if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) munmap(shm_ptr_, kShmSize);
        g_shm_base = nullptr;
        g_shm_size = 0;
    }
};

TEST_F(HmiServerTest, StateMsgAccepted) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateMsg", R"({"cmd":"REFSTATECHANGE","stateMSG":"msg_config"})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_NE(res->body.find("FDBSTATECHANGE"), std::string::npos);
    // 事件真的进了 StateMaster 自身事件环
    EVENT_TYPE ev = EMPTY_EVENT;
    task_.GetEvent(ev);
    EXPECT_EQ(ev, CONFIG_EVENT);
}

TEST_F(HmiServerTest, StateMsgRejectsUnknown) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateMsg", R"({"cmd":"REFSTATECHANGE","stateMSG":"msg_bogus"})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 400);
    EXPECT_NE(res->body.find("ERROR"), std::string::npos);
}

TEST_F(HmiServerTest, FeedbackReturnsStateAndJoints) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateAndPositionFeedback",
                        R"({"cmd":"REFSINGLESOCPOS"})", "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_NE(res->body.find("FDBSINGLESOCPOS"), std::string::npos);
    EXPECT_NE(res->body.find("currentState"), std::string::npos);
    EXPECT_NE(res->body.find("jointAngle"), std::string::npos);
}

TEST_F(HmiServerTest, ExecuteMovementPushesManualQueue) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/executeMovement",
                        R"({"cmd":"FDBMANUALPROGRAM","program":{"unitType":0,"motionType":1,)"
                        R"("is_pose_valid":false,"commandValue":[0,0.1,0.2,0.3,0,0,0]}})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_FALSE(task_.GetManualActionList()->IsEmpty());
}

TEST_F(HmiServerTest, ExecuteProgramPushesProgramQueue) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/executeProgram",
                        R"({"cmd":"TESTAUTOPROGRAM","actionGroup":[[{"name":"a",)"
                        R"("unitType":0,"motionType":1,"is_pose_valid":false,)"
                        R"("speed":{"tcp_m_s":0.5,"ori_rad_s":0.1,"joint_rad_s":0.8,)"
                        R"("ext_rad_s":0.0},"commandValue":[0,0,0,0,0,0,0]}]]})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_FALSE(task_.GetProgramActionList()->IsEmpty());
}

TEST_F(HmiServerTest, ExecuteMovementRejectsInvalidMotionType) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/executeMovement",
                        R"({"cmd":"FDBMANUALPROGRAM","program":{"unitType":0,"motionType":3,)"
                        R"("is_pose_valid":false,"commandValue":[0,0.1,0.2,0.3,0,0,0]}})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 400);
    EXPECT_TRUE(task_.GetManualActionList()->IsEmpty());
}

TEST_F(HmiServerTest, ExecuteProgramRejectsLineWithInvalidMotionType) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/executeProgram",
                        R"({"cmd":"TESTAUTOPROGRAM","actionGroup":[[{"name":"a",)"
                        R"("unitType":0,"motionType":3,"is_pose_valid":false,)"
                        R"("speed":{"tcp_m_s":0.5,"ori_rad_s":0.1,"joint_rad_s":0.8,)"
                        R"("ext_rad_s":0.0},"commandValue":[0,0,0,0,0,0,0]}]]})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);              // 逐行校验：坏行进 info，不是整包 400
    EXPECT_NE(res->body.find("\"accepted\" : 0"), std::string::npos);
    EXPECT_TRUE(task_.GetProgramActionList()->IsEmpty());
}

TEST_F(HmiServerTest, StopProgramSetsFlag) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stopProgram", R"({"StopProgram":true})", "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_TRUE(hmi_.IsStopProgram());
}

TEST_F(HmiServerTest, BadJsonGets400) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateMsg", "{not json", "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 400);
}

// ---- 手动模式（2026-09-08 手动模式设计）----
TEST_F(HmiServerTest, RootReturnsPage) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Get("/");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_NE(res->body.find("BBT HMI"), std::string::npos);
}

TEST_F(HmiServerTest, ModeSwitchRejectedWhenPlanningMissing) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateMsg",
                        R"({"cmd":"REFSTATECHANGE","stateMSG":"msg_auto_mode"})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 409);  // fixture 未注册 planning_task
    EXPECT_NE(res->body.find("planning task not found"), std::string::npos);
}

TEST_F(HmiServerTest, FeedbackContainsRunningMode) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateAndPositionFeedback", "{}", "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    EXPECT_NE(res->body.find("\"runningMode\""), std::string::npos);
    EXPECT_NE(res->body.find("manual"), std::string::npos);  // 默认 MANUAL
}

// ---- 队列水位诊断端点（2026-09-08 部署排障）----
TEST_F(HmiServerTest, QueueStatusReturnsRingLevels) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Get("/queueStatus");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 200);
    for (const char* key : {"manualList", "programList", "trajList", "microList",
                            "currentState", "runningMode", "feedback"}) {
        EXPECT_NE(res->body.find(key), std::string::npos) << "missing " << key;
    }
}

TEST_F(HmiServerTest, CalibZeroRejectedWhenFieldbusMissing) {
    httplib::Client cli("127.0.0.1", kPort);
    auto res = cli.Post("/stateMsg",
                        R"({"cmd":"REFSTATECHANGE","stateMSG":"msg_calib_zero"})",
                        "application/json");
    ASSERT_TRUE(res);
    EXPECT_EQ(res->status, 409);
    EXPECT_NE(res->body.find("fieldbus task not found"), std::string::npos);
}
