#include "task_frame.hpp"
#include "confbase/robot.hpp"
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
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

using namespace robot_config;

extern GlobalData _global_data;

TEST(RobotConfigMultiprocess, ParentBuildsChildReads) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access(kCfgPath, F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    const char* shm_name = "/TEST_ROBOT_CONFIG_MP";
    constexpr size_t kShmSize = 2 * 1024 * 1024;

    shm_unlink(shm_name);
    int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ASSERT_GE(fd, 0);
    ASSERT_EQ(ftruncate(fd, kShmSize), 0);

    void* shm_base = mmap(nullptr, kShmSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    ASSERT_NE(shm_base, MAP_FAILED);

    g_shm_base = shm_base;
    g_shm_size = kShmSize;

    // TF ShmHeader
    struct TfShmHeader {
        uint64_t magic;
        uint32_t version;
        uint32_t size;
        uint32_t crc;
        uint32_t region_table_offset;
    };
    TfShmHeader* tf_header = static_cast<TfShmHeader*>(shm_base);
    tf_header->magic = 0x54462D53484D0000ULL;
    tf_header->version = 1;

    // GlobalData
    GlobalData* gd = reinterpret_cast<GlobalData*>(
        static_cast<char*>(shm_base) + sizeof(TfShmHeader));
    std::memset(gd, 0, sizeof(GlobalData));
    gd->_shared_data_ptr = shm_base;
    gd->_available_data_ptr = static_cast<char*>(shm_base) + 8192;

    _global_data = *gd;
    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) - static_cast<char*>(g_shm_base));

    // Parent builds
    {
        CTaskFrame task;
        task.SetMainTask();
        ASSERT_TRUE(task.BuildRobotConfig(kUrdfPath, kCfgPath));
        // Write updated GlobalData back to shared memory so the child can see it.
        *gd = _global_data;
    }

    pid_t pid = fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        // Child
        g_shm_base = shm_base;
        g_shm_size = kShmSize;
        _global_data = *gd;

        CTaskFrame child_task;
        if (!child_task.AttachRobotConfig()) {
            _exit(1);
        }
        void* base = child_task.GetRobotConfigBase();
        if (!base) {
            _exit(2);
        }
        const robot_config::ShmHeader* header =
            static_cast<const robot_config::ShmHeader*>(base);
        if (header->magic != robot_config::kShmMagic) {
            _exit(3);
        }
        const robot_config::UnitConfig* unit = child_task.GetUnitConfig(0);
        if (!unit || unit->num_joints == 0) {
            _exit(4);
        }
        _exit(0);
    }

    int status;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0) << "Child exit code: " << WEXITSTATUS(status);

    munmap(shm_base, kShmSize);
    shm_unlink(shm_name);
}
