#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <thread>

#include "api_utils.h"
#include "hmi_client.h"
#include "httplib.h"
#include "json.h"
#include "sequence_runner.h"

using namespace web_fsm_tester;

TEST(ApiUtilsTest, ValidStateMsgs) {
    EXPECT_TRUE(IsValidStateMsg("msg_initialize"));
    EXPECT_TRUE(IsValidStateMsg("msg_config"));
    EXPECT_TRUE(IsValidStateMsg("msg_motor_on"));
    EXPECT_TRUE(IsValidStateMsg("msg_motor_off"));
    EXPECT_TRUE(IsValidStateMsg("msg_control_on"));
    EXPECT_TRUE(IsValidStateMsg("msg_control_off"));
    EXPECT_TRUE(IsValidStateMsg("msg_error"));
    EXPECT_TRUE(IsValidStateMsg("msg_reset_ok"));
    EXPECT_TRUE(IsValidStateMsg("msg_estop"));
    EXPECT_FALSE(IsValidStateMsg("msg_bogus"));
    EXPECT_FALSE(IsValidStateMsg(""));
    EXPECT_FALSE(IsValidStateMsg("MSG_CONFIG"));
}

TEST(ApiUtilsTest, StatusNames) {
    EXPECT_STREQ(StatusName(0), "UNINITIATED");
    EXPECT_STREQ(StatusName(1), "UNCONFIGED");
    EXPECT_STREQ(StatusName(2), "MOTOR_OFF");
    EXPECT_STREQ(StatusName(3), "READY");
    EXPECT_STREQ(StatusName(4), "RUNNING");
    EXPECT_STREQ(StatusName(5), "EMERGENCY_STOP");
    EXPECT_STREQ(StatusName(127), "ERROR");
    EXPECT_STREQ(StatusName(-1), "UNKNOWN");
    EXPECT_STREQ(StatusName(99), "UNKNOWN");
}

TEST(ApiUtilsTest, BuildStatusJsonRoundTrip) {
    std::vector<TaskStatusInfo> tasks = {
        {"state_master", 1}, {"planning_task", 1},
        {"servo_task", 1}, {"fieldbus_task", 0}};
    SeqView seq;
    seq.active = true;
    seq.step = 2;
    seq.label = "CONFIG";

    std::string json = BuildStatusJson(true, tasks, seq);

    Json::Value root;
    Json::CharReaderBuilder rb;
    std::string errs;
    std::unique_ptr<Json::CharReader> reader(rb.newCharReader());
    ASSERT_TRUE(reader->parse(json.data(), json.data() + json.size(), &root, &errs));
    EXPECT_TRUE(root["running"].asBool());
    ASSERT_EQ(root["tasks"].size(), 4u);
    EXPECT_EQ(root["tasks"][0]["name"].asString(), "state_master");
    EXPECT_EQ(root["tasks"][0]["status"].asInt(), 1);
    EXPECT_EQ(root["tasks"][0]["statusName"].asString(), "UNCONFIGED");
    EXPECT_EQ(root["tasks"][3]["statusName"].asString(), "UNINITIATED");
    EXPECT_TRUE(root["seq"]["active"].asBool());
    EXPECT_EQ(root["seq"]["step"].asInt(), 2);
    EXPECT_EQ(root["seq"]["label"].asString(), "CONFIG");
    EXPECT_EQ(root["seq"]["stepCount"].asInt(), 8);
}

TEST(SequenceStepsTest, GoldenPathTable) {
    const auto& steps = SequenceSteps();
    ASSERT_EQ(steps.size(), 8u);
    EXPECT_STREQ(steps[0].msg, "msg_initialize");
    EXPECT_EQ(steps[0].expected_status, UNCONFIGED);
    EXPECT_EQ(steps[0].timeout_ms, 25000);   // INIT 含 EtherCAT 冷启动
    EXPECT_STREQ(steps[1].msg, "msg_config");
    EXPECT_EQ(steps[1].expected_status, MOTOR_OFF);
    EXPECT_STREQ(steps[2].msg, "msg_motor_on");
    EXPECT_EQ(steps[2].expected_status, READY);
    EXPECT_STREQ(steps[3].msg, "msg_control_on");
    EXPECT_EQ(steps[3].expected_status, RUNNING);
    EXPECT_STREQ(steps[4].msg, "msg_control_off");
    EXPECT_EQ(steps[4].expected_status, READY);
    EXPECT_STREQ(steps[5].msg, "msg_motor_off");
    EXPECT_EQ(steps[5].expected_status, MOTOR_OFF);
    EXPECT_STREQ(steps[6].msg, "msg_error");
    EXPECT_EQ(steps[6].expected_status, ERROR);
    EXPECT_STREQ(steps[7].msg, "msg_reset_ok");
    EXPECT_EQ(steps[7].expected_status, MOTOR_OFF);
    for (const auto& s : steps) {
        EXPECT_TRUE(IsValidStateMsg(s.msg)) << s.msg;
        EXPECT_GT(s.timeout_ms, 5100) << s.msg;   // 必须大于主任务默认同步窗口
        EXPECT_NE(s.label, nullptr);
    }
}

TEST(HmiClientTest, GetPositionFeedbackNoServer) {
    HmiClient hmi(19999);  // 假定该端口无人监听
    std::string body, err;
    EXPECT_FALSE(hmi.GetPositionFeedback(&body, &err));
    EXPECT_FALSE(err.empty());
}

TEST(HmiClientTest, GetPositionFeedbackRelaysBody) {
    httplib::Server svr;
    svr.Post("/stateAndPositionFeedback",
             [](const httplib::Request&, httplib::Response& res) {
                 res.set_content(
                     R"({"cmd":"FDBSINGLESOCPOS","currentState":"READY",)"
                     R"("socFeedbacks":[{"jointAngle":{"J1":0.5,"J2":-0.25}}]})",
                     "application/json");
             });
    int port = svr.bind_to_any_port("127.0.0.1");
    ASSERT_GT(port, 0);
    std::thread t([&] { svr.listen_after_bind(); });

    HmiClient hmi(port);
    std::string body, err;
    EXPECT_TRUE(hmi.GetPositionFeedback(&body, &err)) << err;
    EXPECT_NE(body.find("FDBSINGLESOCPOS"), std::string::npos);
    EXPECT_NE(body.find("J1"), std::string::npos);

    svr.stop();
    t.join();
}
