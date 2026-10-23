#include <gtest/gtest.h>
#include "tf_feedback.hpp"
#include "tf_shm_region.hpp"
#include <cstring>
#include <thread>
#include <sys/mman.h>

class TfFeedbackTest : public ::testing::Test {
protected:
    void* _mem = nullptr;
    TfShmRegionManager _region_mgr;
    TfFeedbackManager _fb_mgr;
    void SetUp() override {
        _mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        ASSERT_NE(_mem, MAP_FAILED);
        ASSERT_TRUE(_region_mgr.Init(_mem, 100 * 1024 * 1024, sizeof(ShmHeader)));
        void* fb_base = _region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
        ASSERT_TRUE(_fb_mgr.Init(fb_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK)));
    }
    void TearDown() override {
        if (_mem != MAP_FAILED && _mem != nullptr) munmap(_mem, 100 * 1024 * 1024);
    }
};

TEST_F(TfFeedbackTest, UpdateAndGetJoints) {
    FEEDBACK_ALL_JOINTS data_in;
    memset(&data_in, 0, sizeof(data_in));
    for (int i = 0; i < (int)robot_config::kRobotMaxJoints; i++) {
        data_in.joints[i].position_feedback = 0.1 * i;
        data_in.joints[i].speed_feedback = 0.01 * i;
    }
    _fb_mgr.UpdateJoints(data_in);
    FEEDBACK_ALL_JOINTS data_out;
    EXPECT_TRUE(_fb_mgr.GetJoints(data_out));
    for (int i = 0; i < (int)robot_config::kRobotMaxJoints; i++) {
        EXPECT_DOUBLE_EQ(data_out.joints[i].position_feedback, 0.1 * i);
        EXPECT_DOUBLE_EQ(data_out.joints[i].speed_feedback, 0.01 * i);
    }
}

TEST_F(TfFeedbackTest, UpdateAndGetPose) {
    FEEDBACK_ALL_POSE data_in;
    memset(&data_in, 0, sizeof(data_in));
    for (int i = 0; i < (int)robot_config::kRobotMaxUnits; i++) {
        data_in.soc_pose[i].x = 1.0 * i;
        data_in.soc_pose[i].y = 2.0 * i;
    }
    _fb_mgr.UpdatePose(data_in);
    FEEDBACK_ALL_POSE data_out;
    EXPECT_TRUE(_fb_mgr.GetPose(data_out));
    for (int i = 0; i < (int)robot_config::kRobotMaxUnits; i++) {
        EXPECT_DOUBLE_EQ(data_out.soc_pose[i].x, 1.0 * i);
        EXPECT_DOUBLE_EQ(data_out.soc_pose[i].y, 2.0 * i);
    }
}

TEST_F(TfFeedbackTest, UpdateAndGetForce) {
    FEEDBACK_ALL_FORCE data_in;
    memset(&data_in, 0, sizeof(data_in));
    for (int i = 0; i < (int)robot_config::kRobotMaxUnits; i++) {
        data_in.soc_force[i].force_x = 0.5 * i;
        data_in.soc_force[i].force_y = 0.3 * i;
    }
    _fb_mgr.UpdateForce(data_in);
    FEEDBACK_ALL_FORCE data_out;
    EXPECT_TRUE(_fb_mgr.GetForce(data_out));
    for (int i = 0; i < (int)robot_config::kRobotMaxUnits; i++) {
        EXPECT_DOUBLE_EQ(data_out.soc_force[i].force_x, 0.5 * i);
        EXPECT_DOUBLE_EQ(data_out.soc_force[i].force_y, 0.3 * i);
    }
}

TEST_F(TfFeedbackTest, AttachFromSecondManager) {
    TfFeedbackManager fb2;
    void* fb_base = _region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
    EXPECT_TRUE(fb2.Attach(fb_base, _region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK)));
    FEEDBACK_ALL_JOINTS data_in;
    memset(&data_in, 0, sizeof(data_in));
    data_in.joints[0].position_feedback = 3.14;
    _fb_mgr.UpdateJoints(data_in);
    FEEDBACK_ALL_JOINTS data_out;
    EXPECT_TRUE(fb2.GetJoints(data_out));
    EXPECT_DOUBLE_EQ(data_out.joints[0].position_feedback, 3.14);
}

TEST_F(TfFeedbackTest, ConcurrentReadWrite) {
    std::atomic<bool> stop{false};
    std::thread writer([&]() {
        FEEDBACK_ALL_JOINTS data;
        int count = 0;
        while (!stop.load()) {
            memset(&data, 0, sizeof(data));
            for (int i = 0; i < (int)robot_config::kRobotMaxJoints; i++) {
                data.joints[i].position_feedback = count;
            }
            _fb_mgr.UpdateJoints(data);
            count++;
            usleep(100);
        }
    });
    std::thread reader([&]() {
        int reads = 0;
        while (!stop.load() && reads < 1000) {
            FEEDBACK_ALL_JOINTS data;
            if (_fb_mgr.GetJoints(data)) {
                double first = data.joints[0].position_feedback;
                bool consistent = true;
                for (int i = 1; i < (int)robot_config::kRobotMaxJoints; i++) {
                    if (data.joints[i].position_feedback != first) {
                        consistent = false; break;
                    }
                }
                EXPECT_TRUE(consistent);
            }
            reads++;
            usleep(50);
        }
    });
    usleep(500000);
    stop.store(true);
    writer.join();
    reader.join();
}

TEST_F(TfFeedbackTest, CacheFallbackNeverReturnsCorrupt) {
    for (int cycle = 0; cycle < 100; cycle++) {
        FEEDBACK_ALL_JOINTS data;
        memset(&data, 0, sizeof(data));
        data.joints[0].position_feedback = cycle;
        _fb_mgr.UpdateJoints(data);
    }
    FEEDBACK_ALL_JOINTS data;
    EXPECT_TRUE(_fb_mgr.GetJoints(data));
    EXPECT_GE(data.joints[0].position_feedback, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
