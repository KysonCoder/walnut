#include "task_frame.hpp"
#include "confbase/robot.hpp"
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#if ROBOT_TYPE == HUMANOID
constexpr const char* kUrdfPath = "/home/user/claude/cfg/humanoid.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/humanoid_robot_conf.cfg";
#else
constexpr const char* kUrdfPath = "/home/user/claude/cfg/2arm_v1.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/dualarm_robot_conf.cfg";
#endif

extern GlobalData _global_data;

class JointPosTest : public ::testing::Test {
 protected:
    void* shm_base = nullptr;
    static constexpr size_t kTestShmSize = 1024 * 1024;

    struct TfShmHeader {
        uint64_t magic = 0x54462D53484D0000ULL;
        uint32_t version = 1;
        uint32_t size = 0;
        uint32_t crc = 0;
        uint32_t region_table_offset = 0;
    };

    void SetUp() override {
        int fd = shm_open("/TEST_ROBOT_CONFIG_JOINTPOS", O_CREAT | O_RDWR, 0666);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(ftruncate(fd, kTestShmSize), 0);
        shm_base = mmap(nullptr, kTestShmSize, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd, 0);
        close(fd);
        ASSERT_NE(shm_base, MAP_FAILED);

        g_shm_base = shm_base;
        g_shm_size = kTestShmSize;

        TfShmHeader* tf_header = static_cast<TfShmHeader*>(shm_base);
        *tf_header = TfShmHeader{};
        tf_header->magic = 0x54462D53484D0000ULL;

        GlobalData* gd = reinterpret_cast<GlobalData*>(
            static_cast<char*>(shm_base) + sizeof(TfShmHeader));
        std::memset(gd, 0, sizeof(GlobalData));
        gd->_shared_data_ptr = shm_base;
        gd->_available_data_ptr = static_cast<char*>(shm_base) + 8192;
        gd->_global_mutex = nullptr;
        gd->_task_count = nullptr;

        _global_data = *gd;
    }

    void TearDown() override {
        if (shm_base) munmap(shm_base, kTestShmSize);
        shm_unlink("/TEST_ROBOT_CONFIG_JOINTPOS");
    }
};

TEST_F(JointPosTest, MakeJointPosBindsMapAndParams) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access(kCfgPath, F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) -
        static_cast<char*>(g_shm_base));

    CTaskFrame main_task;
    main_task.SetMainTask();
    ASSERT_TRUE(main_task.BuildRobotConfig(kUrdfPath, kCfgPath));

    GlobalData* gd = reinterpret_cast<GlobalData*>(
        static_cast<char*>(shm_base) + sizeof(TfShmHeader));
    *gd = _global_data;

    CTaskFrame child_task;
    ASSERT_TRUE(child_task.AttachRobotConfig());

    double data[robot_config::kRobotMaxJoints] = {};
    CJointPos jp = child_task.MakeJointPos(data);
    EXPECT_TRUE(jp.IsAttached());

    jp.SetLogical(0, 1.0);
    EXPECT_DOUBLE_EQ(jp.GetLogical(0), 1.0);

    jp.Set(0, 3.0);
    EXPECT_DOUBLE_EQ(jp.Get(0), 3.0);
}

TEST_F(JointPosTest, MakeJointPosWithoutRobotConfigDataOnly) {
    CTaskFrame task;
    double data[robot_config::kRobotMaxJoints] = {};
    CJointPos jp = task.MakeJointPos(data);
    EXPECT_TRUE(jp.IsAttached());
    // Without robot config, map and params are null but raw data ops still work.
    jp.Set(0, 2.0);
    EXPECT_DOUBLE_EQ(jp.Get(0), 2.0);
}

TEST_F(JointPosTest, MakeJointPosNullptrReturnsDetached) {
    CTaskFrame task;
    CJointPos jp = task.MakeJointPos(nullptr);
    EXPECT_FALSE(jp.IsAttached());
}
