#include <gtest/gtest.h>
#include "tf_msg_queue.hpp"
#include "tf_shm_region.hpp"
#include <cstring>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class TfMsgQueueForkTest : public ::testing::Test {
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

TEST_F(TfMsgQueueForkTest, CrossProcessFork) {
    void* mq_base = _region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
    size_t mq_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE);

    pid_t pid = fork();
    if (pid == 0) {
        // Child process: attach and send message
        TfMsgQueueManager child_mq;
        ASSERT_TRUE(child_mq.Attach(mq_base, mq_size));
        ASSERT_TRUE(child_mq.Send(0, MsgType::CMD_JOG, "jog", 4));
        _exit(0);
    }

    ASSERT_NE(pid, -1);
    int status;
    waitpid(pid, &status, 0);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);

    // Parent verifies child wrote data
    Message out;
    EXPECT_TRUE(_mq_mgr.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::CMD_JOG);
    EXPECT_STREQ(out.payload, "jog");
}

TEST_F(TfMsgQueueForkTest, CrossProcessSendByName) {
    void* mq_base = _region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
    size_t mq_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE);

    // 写端按 task_id 注册名字
    ASSERT_TRUE(_mq_mgr.RegisterQueue(0, "child"));

    pid_t pid = fork();
    if (pid == 0) {
        // 子进程（读端）可见注册表
        TfMsgQueueManager child_mq;
        if (!child_mq.Attach(mq_base, mq_size)) _exit(1);
        if (child_mq.FindQueue("child") != 0) _exit(2);
        _exit(0);
    }

    ASSERT_NE(pid, -1);
    int status;
    waitpid(pid, &status, 0);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);

    // 父进程按名发送，按 id 接收
    ASSERT_TRUE(_mq_mgr.Send("child", MsgType::PROGRAM_ACTION, "run", 4));
    Message out;
    ASSERT_TRUE(_mq_mgr.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::PROGRAM_ACTION);
    EXPECT_STREQ(out.payload, "run");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
