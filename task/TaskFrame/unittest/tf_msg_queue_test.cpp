#include <gtest/gtest.h>
#include "tf_msg_queue.hpp"
#include "tf_shm_region.hpp"
#include <cstring>
#include <sys/mman.h>

class TfMsgQueueTest : public ::testing::Test {
protected:
    void* _mem = nullptr;
    TfShmRegionManager _region_mgr;
    TfMsgQueueManager _mq_mgr;
    void SetUp() override {
        _mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_mem, MAP_FAILED);
        ASSERT_TRUE(_region_mgr.Init(_mem, 100 * 1024 * 1024, sizeof(ShmHeader)));
        void* mq_base = _region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
        ASSERT_TRUE(_mq_mgr.Init(mq_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE), 4));
    }
    void TearDown() override {
        if (_mem != MAP_FAILED && _mem != nullptr) munmap(_mem, 100 * 1024 * 1024);
    }
};

TEST_F(TfMsgQueueTest, SendAndReceive) {
    Message msg; msg.type = MsgType::AUTO_MODE; msg.priority = 5;
    strcpy(msg.payload, "test_data"); msg.payload_size = 10;
    EXPECT_TRUE(_mq_mgr.Send(0, msg.type, msg.payload, msg.payload_size, msg.priority));
    Message out;
    EXPECT_TRUE(_mq_mgr.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::AUTO_MODE);
    EXPECT_EQ(out.priority, 5u);
    EXPECT_STREQ(out.payload, "test_data");
}

TEST_F(TfMsgQueueTest, ReceiveEmptyReturnsFalse) {
    Message out; EXPECT_FALSE(_mq_mgr.Receive(0, out));
}

TEST_F(TfMsgQueueTest, MultiTaskIsolation) {
    _mq_mgr.Send(0, MsgType::MANUAL_MODE, nullptr, 0);
    _mq_mgr.Send(1, MsgType::AUTO_MODE, nullptr, 0);
    Message out0, out1;
    EXPECT_TRUE(_mq_mgr.Receive(0, out0));
    EXPECT_TRUE(_mq_mgr.Receive(1, out1));
    EXPECT_EQ(out0.type, MsgType::MANUAL_MODE);
    EXPECT_EQ(out1.type, MsgType::AUTO_MODE);
}

TEST_F(TfMsgQueueTest, SeqnumIncrements) {
    _mq_mgr.Send(0, MsgType::PROGRAM_ACTION, nullptr, 0);
    _mq_mgr.Send(0, MsgType::PROGRAM_ACTION, nullptr, 0);
    Message out1, out2;
    EXPECT_TRUE(_mq_mgr.Receive(0, out1));
    EXPECT_TRUE(_mq_mgr.Receive(0, out2));
    EXPECT_EQ(out1.seqnum + 1, out2.seqnum);
}

TEST_F(TfMsgQueueTest, AttachFromSecondManager) {
    TfMsgQueueManager mq2;
    void* mq_base = _region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
    EXPECT_TRUE(mq2.Attach(mq_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE)));
    _mq_mgr.Send(0, MsgType::CMD_STOP, nullptr, 0);
    Message out;
    EXPECT_TRUE(mq2.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::CMD_STOP);
}

TEST_F(TfMsgQueueTest, RegistryRegisterAndFind) {
    EXPECT_TRUE(_mq_mgr.RegisterQueue(0, "state_master"));
    EXPECT_TRUE(_mq_mgr.RegisterQueue(1, "planning"));
    EXPECT_EQ(_mq_mgr.FindQueue("planning"), 1);
    EXPECT_EQ(_mq_mgr.FindQueue("ghost"), -1);
    EXPECT_FALSE(_mq_mgr.RegisterQueue(2, "planning"));  // 同名不同 id
}

TEST_F(TfMsgQueueTest, SendByName) {
    ASSERT_TRUE(_mq_mgr.RegisterQueue(1, "planning"));
    const char* payload = "goto";
    ASSERT_TRUE(_mq_mgr.Send("planning", MsgType::PROGRAM_ACTION, payload, 5));
    Message out;
    ASSERT_TRUE(_mq_mgr.Receive(1, out));
    EXPECT_EQ(out.type, MsgType::PROGRAM_ACTION);
    EXPECT_STREQ(out.payload, "goto");
}

TEST_F(TfMsgQueueTest, RingFullReturnsFalseNoLoss) {
    for (int i = 0; i < 32; ++i) {
        ASSERT_TRUE(_mq_mgr.Send(0, MsgType::CMD_JOG)) << "send " << i;
    }
    EXPECT_FALSE(_mq_mgr.Send(0, MsgType::CMD_STOP));
    Message out;
    ASSERT_TRUE(_mq_mgr.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::CMD_JOG);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
