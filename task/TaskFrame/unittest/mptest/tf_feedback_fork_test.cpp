#include <gtest/gtest.h>
#include "tf_feedback.hpp"
#include "tf_shm_region.hpp"
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class TfFeedbackForkTest : public ::testing::Test {
protected:
    void* _mem = nullptr;
    TfShmRegionManager _region_mgr;
    TfFeedbackManager _fb_mgr;
    void SetUp() override {
        _mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_mem, MAP_FAILED);
        ASSERT_TRUE(_region_mgr.Init(_mem, 100 * 1024 * 1024, sizeof(ShmHeader)));
        void* fb_base = _region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
        ASSERT_TRUE(_fb_mgr.Init(fb_base,
            _region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK)));
    }
    void TearDown() override {
        if (_mem != MAP_FAILED && _mem != nullptr) munmap(_mem, 100 * 1024 * 1024);
    }
};

TEST_F(TfFeedbackForkTest, CrossProcessSeqlockConsistency) {
    void* fb_base = _region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
    size_t fb_size = _region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK);

    pid_t pid = fork();
    if (pid == 0) {
        // 子进程：写端，高频写 1000 帧
        TfFeedbackManager child;
        if (!child.Attach(fb_base, fb_size)) _exit(1);
        FEEDBACK_ALL_JOINTS data = {};
        for (int i = 0; i < 1000; ++i) {
            data.joints[0].position_feedback = static_cast<double>(i);
            data.joints[0].torque_feedback = static_cast<double>(i) * 2.0;
            child.UpdateJoints(data);
        }
        _exit(0);
    }

    ASSERT_NE(pid, -1);
    // 父进程：读端，读到的单帧内 position*2 == torque（seqlock 一致性）
    bool got_any = false;
    for (int retry = 0; retry < 10000; ++retry) {
        FEEDBACK_ALL_JOINTS out;
        if (_fb_mgr.GetJoints(out)) {
            got_any = true;
            EXPECT_DOUBLE_EQ(out.joints[0].position_feedback * 2.0,
                             out.joints[0].torque_feedback);
        }
    }
    EXPECT_TRUE(got_any);

    int status;
    waitpid(pid, &status, 0);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
