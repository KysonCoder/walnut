#include <gtest/gtest.h>
#include "tf_event.hpp"
#include "tf_shm_region.hpp"
#include <cstring>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class TfEventForkTest : public ::testing::Test {
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

TEST_F(TfEventForkTest, CrossProcessFork) {
    void* evt_base = _region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    size_t evt_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT);

    pid_t pid = fork();
    if (pid == 0) {
        // Child process: attach and publish event
        TfEventManager child_mgr;
        ASSERT_TRUE(child_mgr.Attach(evt_base, evt_size));
        ASSERT_TRUE(child_mgr.PublishEvent(0, EventType::CONFIG, "child", 6));
        _exit(0);
    }

    ASSERT_NE(pid, -1);
    int status;
    waitpid(pid, &status, 0);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);

    // Parent verifies child wrote data
    Event out;
    EXPECT_TRUE(_evt_mgr.ConsumeEvent(0, out));
    EXPECT_EQ(out.type, EventType::CONFIG);
    EXPECT_STREQ(out.payload, "child");
}

TEST_F(TfEventForkTest, CrossProcessAck) {
    void* evt_base = _region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    size_t evt_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT);

    // 写端注册并发布带 REQUIRE_ACK 的事件
    ASSERT_TRUE(_evt_mgr.RegisterTask(0, "child"));
    ASSERT_TRUE(_evt_mgr.PublishEvent(0, EventType::CONFIG, nullptr, 0,
                                      EVENT_FLAG_REQUIRE_ACK));

    pid_t pid = fork();
    if (pid == 0) {
        TfEventManager child_mgr;
        if (!child_mgr.Attach(evt_base, evt_size)) _exit(1);
        // 读端禁止广播（SPSC 防护）
        if (child_mgr.PublishBroadcast(EventType::ERROR)) _exit(2);
        Event ev;
        if (!child_mgr.ConsumeEvent(0, ev)) _exit(3);
        if (!(ev.flags & EVENT_FLAG_REQUIRE_ACK)) _exit(4);
        child_mgr.SendAck(0, ev.seqnum);
        _exit(0);
    }

    ASSERT_NE(pid, -1);
    int status;
    waitpid(pid, &status, 0);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);
    EXPECT_TRUE(_evt_mgr.WaitAck(0, 1, 1000));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
