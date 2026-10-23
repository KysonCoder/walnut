#include <gtest/gtest.h>
#include "tf_event.hpp"
#include "tf_shm_region.hpp"
#include <cstring>
#include <sys/mman.h>

class TfEventTest : public ::testing::Test {
protected:
    void* _mem = nullptr;
    TfShmRegionManager _region_mgr;
    TfEventManager _evt_mgr;
    void SetUp() override {
        _mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_mem, MAP_FAILED);
        ASSERT_TRUE(_region_mgr.Init(_mem, 100 * 1024 * 1024, sizeof(ShmHeader)));
        void* evt_base = _region_mgr.GetRegionBase(TfShmRegionId::EVENT);
        ASSERT_TRUE(_evt_mgr.Init(evt_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT), 4));
    }
    void TearDown() override {
        if (_mem != MAP_FAILED && _mem != nullptr) munmap(_mem, 100 * 1024 * 1024);
    }
};

TEST_F(TfEventTest, PublishAndConsume) {
    Event ev; ev.type = EventType::INIT; ev.seqnum = 1; strcpy(ev.payload, "hello");
    EXPECT_TRUE(_evt_mgr.PublishEvent(0, EventType::INIT, ev.payload, strlen(ev.payload) + 1));
    Event out;
    EXPECT_TRUE(_evt_mgr.ConsumeEvent(0, out));
    EXPECT_EQ(out.type, EventType::INIT);
    EXPECT_EQ(out.seqnum, 1u);
    EXPECT_STREQ(out.payload, "hello");
}

TEST_F(TfEventTest, ConsumeEmptyReturnsFalse) {
    Event out;
    EXPECT_FALSE(_evt_mgr.ConsumeEvent(0, out));
}

TEST_F(TfEventTest, BroadcastReachesAllTasks) {
    EXPECT_TRUE(_evt_mgr.PublishBroadcast(EventType::MOTOR_ON));
    for (int i = 0; i < 4; i++) {
        EXPECT_TRUE(_evt_mgr.HasEvent(i));
        Event out;
        EXPECT_TRUE(_evt_mgr.ConsumeEvent(i, out));
        EXPECT_EQ(out.type, EventType::MOTOR_ON);
    }
}

TEST_F(TfEventTest, SeqnumIncrements) {
    _evt_mgr.PublishEvent(0, EventType::INIT);
    _evt_mgr.PublishEvent(0, EventType::CONFIG);
    Event out1, out2;
    EXPECT_TRUE(_evt_mgr.ConsumeEvent(0, out1));
    EXPECT_TRUE(_evt_mgr.ConsumeEvent(0, out2));
    EXPECT_EQ(out1.seqnum + 1, out2.seqnum);
}

TEST_F(TfEventTest, AttachFromSecondManager) {
    TfEventManager mgr2;
    void* evt_base = _region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    EXPECT_TRUE(mgr2.Attach(evt_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT)));
    _evt_mgr.PublishEvent(0, EventType::ERROR);
    Event out;
    EXPECT_TRUE(mgr2.ConsumeEvent(0, out));
    EXPECT_EQ(out.type, EventType::ERROR);
}

TEST_F(TfEventTest, RegistryRegisterAndFind) {
    EXPECT_TRUE(_evt_mgr.RegisterTask(0, "state_master"));
    EXPECT_TRUE(_evt_mgr.RegisterTask(1, "servo"));
    EXPECT_EQ(_evt_mgr.FindTask("servo"), 1);
    EXPECT_EQ(_evt_mgr.FindTask("nonexistent"), -1);
    // 同名不同 id → 拒绝
    EXPECT_FALSE(_evt_mgr.RegisterTask(2, "servo"));
    // 幂等：同名同 id → true
    EXPECT_TRUE(_evt_mgr.RegisterTask(1, "servo"));
}

TEST_F(TfEventTest, PublishByName) {
    ASSERT_TRUE(_evt_mgr.RegisterTask(1, "servo"));
    ASSERT_TRUE(_evt_mgr.PublishEvent("servo", EventType::MOTOR_ON));
    Event out;
    ASSERT_TRUE(_evt_mgr.ConsumeEvent(1, out));
    EXPECT_EQ(out.type, EventType::MOTOR_ON);
}

TEST_F(TfEventTest, RingFullReturnsFalseNoLoss) {
    for (int i = 0; i < 16; ++i) {
        ASSERT_TRUE(_evt_mgr.PublishEvent(0, EventType::CONFIG)) << "push " << i;
    }
    EXPECT_FALSE(_evt_mgr.PublishEvent(0, EventType::ERROR));  // 满 → false，不覆盖
    Event out;
    ASSERT_TRUE(_evt_mgr.ConsumeEvent(0, out));
    EXPECT_EQ(out.type, EventType::CONFIG);  // 最早的仍在
}

TEST_F(TfEventTest, BroadcastRejectedForReader) {
    void* evt_base = _region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    size_t evt_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT);
    TfEventManager reader;
    ASSERT_TRUE(reader.Attach(evt_base, evt_size));
    EXPECT_FALSE(reader.IsWriter());
    EXPECT_FALSE(reader.PublishBroadcast(EventType::ERROR));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
