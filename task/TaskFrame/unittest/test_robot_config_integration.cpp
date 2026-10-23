#include "task_frame.hpp"
#include "confbase/robot.hpp"
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <vector>
#include <cstdio>

#if ROBOT_TYPE == HUMANOID
constexpr const char* kUrdfPath = "/home/user/claude/cfg/humanoid.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/humanoid_robot_conf.cfg";
#else
constexpr const char* kUrdfPath = "/home/user/claude/cfg/2arm_v1.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/dualarm_robot_conf.cfg";
#endif

using namespace robot_config;

extern GlobalData _global_data;

class RobotConfigIntegrationTest : public ::testing::Test {
protected:
    struct TfShmHeader {
        uint64_t magic = 0x54462D53484D0000ULL;
        uint32_t version = 1;
        uint32_t size = 0;
        uint32_t crc = 0;
        uint32_t region_table_offset = 0;
    };

    void* shm_base = nullptr;
    static constexpr size_t kTestShmSize = 1024 * 1024;

    void SetUp() override {
        int fd = shm_open("/TEST_ROBOT_CONFIG", O_CREAT | O_RDWR, 0666);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(ftruncate(fd, kTestShmSize), 0);
        shm_base = mmap(nullptr, kTestShmSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        ASSERT_NE(shm_base, MAP_FAILED);

        g_shm_base = shm_base;
        g_shm_size = kTestShmSize;

        // Write TF ShmHeader
        TfShmHeader* tf_header = static_cast<TfShmHeader*>(shm_base);
        *tf_header = TfShmHeader{};
        tf_header->magic = 0x54462D53484D0000ULL;

        // GlobalData after TF ShmHeader
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
        robot_config::RobotConfigStore::Instance().Detach();
        if (shm_base) munmap(shm_base, kTestShmSize);
        shm_unlink("/TEST_ROBOT_CONFIG");
    }
};

TEST_F(RobotConfigIntegrationTest, BuildCreatesValidHeader) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access(kCfgPath, F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) - static_cast<char*>(g_shm_base));

    CTaskFrame task;
    task.SetMainTask();
    bool ok = task.BuildRobotConfig(kUrdfPath, kCfgPath);
    EXPECT_TRUE(ok) << "BuildRobotConfig failed";

    void* rc_base = task.GetRobotConfigBase();
    ASSERT_NE(rc_base, nullptr);
    EXPECT_TRUE(task.IsRobotConfigReady());

    const robot_config::ShmHeader* rc_header =
        static_cast<const robot_config::ShmHeader*>(rc_base);
    EXPECT_EQ(rc_header->magic, robot_config::kShmMagic);
    EXPECT_EQ(rc_header->version, robot_config::kShmVersion);
    EXPECT_EQ(rc_header->page_size, robot_config::kShmPageSize);
    EXPECT_GT(rc_header->num_units, 0u);
}

TEST_F(RobotConfigIntegrationTest, ChildCanAttachAfterParentBuilds) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access(kCfgPath, F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    // Parent builds
    {
        _global_data._robot_config = ShmPtr<char>(
            static_cast<char*>(_global_data._available_data_ptr) - static_cast<char*>(g_shm_base));
        CTaskFrame main_task;
        main_task.SetMainTask();
        ASSERT_TRUE(main_task.BuildRobotConfig(kUrdfPath, kCfgPath));
        // Write updated GlobalData back to shared memory so the child can see it.
        GlobalData* gd = reinterpret_cast<GlobalData*>(
            static_cast<char*>(shm_base) + sizeof(TfShmHeader));
        *gd = _global_data;
    }

    // Child attaches
    {
        CTaskFrame child_task;
        ASSERT_TRUE(child_task.AttachRobotConfig());
        EXPECT_TRUE(child_task.IsRobotConfigReady());

        void* base = child_task.GetRobotConfigBase();
        ASSERT_NE(base, nullptr);
        EXPECT_TRUE(child_task.IsRobotConfigReady());
        const robot_config::ShmHeader* header =
            static_cast<const robot_config::ShmHeader*>(base);
        EXPECT_EQ(header->magic, robot_config::kShmMagic);
    }
}

// Regression test: BuildToMemory() output must be byte-identical to the
// legacy Build(path) output for the same inputs. This validates that
// TaskFrame's new zero-file-I/O path produces the same shared-memory layout
// as the previous temporary-file bridge.
TEST_F(RobotConfigIntegrationTest, BuildToMemoryMatchesBuildFile) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access(kCfgPath, F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    const char* tmp_path = "/tmp/rc_build_to_memory_cmp.bin";

    // Build via legacy file path
    {
        RobotConfigBuilder builder;
        ASSERT_TRUE(builder.Init());
        ASSERT_TRUE(builder.LoadDriveParams(kCfgPath));
        ASSERT_TRUE(builder.LoadUrdfKinematics(kUrdfPath));
        ASSERT_TRUE(builder.Build(tmp_path));
    }

    // Read file content
    std::ifstream file(tmp_path, std::ios::binary | std::ios::ate);
    ASSERT_TRUE(file.is_open());
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> file_buffer(file_size);
    ASSERT_TRUE(file.read(file_buffer.data(), file_size));
    file.close();
    std::remove(tmp_path);

    // Build via BuildToMemory into a page-aligned buffer
    size_t serialized_size = 0;
    {
        RobotConfigBuilder builder;
        ASSERT_TRUE(builder.Init());
        ASSERT_TRUE(builder.LoadDriveParams(kCfgPath));
        ASSERT_TRUE(builder.LoadUrdfKinematics(kUrdfPath));
        serialized_size = builder.GetSerializedSize();
        ASSERT_GT(serialized_size, 0u);
        ASSERT_EQ(serialized_size, static_cast<size_t>(file_size));
    }

    void* mem_buffer = nullptr;
    ASSERT_EQ(posix_memalign(&mem_buffer, kShmPageSize, serialized_size), 0);
    ASSERT_NE(mem_buffer, nullptr);

    {
        RobotConfigBuilder builder;
        ASSERT_TRUE(builder.Init());
        ASSERT_TRUE(builder.LoadDriveParams(kCfgPath));
        ASSERT_TRUE(builder.LoadUrdfKinematics(kUrdfPath));
        size_t written = 0;
        ASSERT_TRUE(builder.BuildToMemory(mem_buffer, serialized_size, &written));
        EXPECT_EQ(written, serialized_size);
    }

    EXPECT_EQ(std::memcmp(mem_buffer, file_buffer.data(), serialized_size), 0)
        << "BuildToMemory output differs from Build(path) output";

    free(mem_buffer);
}

