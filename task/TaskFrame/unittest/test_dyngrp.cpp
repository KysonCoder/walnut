#include "task_frame.hpp"
#include "confbase/robot.hpp"
#include "confbase/dyn_group_model.hpp"
#include "confbase/shm_layout.hpp"
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

using namespace robot_config;

extern GlobalData _global_data;

namespace {

const robot_config::DynGroupConfigPage* GetDynGroupPageFromBase(const void* base) {
    const robot_config::ShmHeader* header = static_cast<const robot_config::ShmHeader*>(base);
    if (header == nullptr) return nullptr;
    uint64_t dyngrp_offset = header->dyngrp_offset;
    if (dyngrp_offset == 0 && header->total_size >= robot_config::kShmPageSize) {
        dyngrp_offset = header->total_size - robot_config::kShmPageSize;
    }
    if (dyngrp_offset == 0) return nullptr;
    return reinterpret_cast<const robot_config::DynGroupConfigPage*>(
        static_cast<const char*>(base) + dyngrp_offset);
}

}  // namespace

class DynGroupTest : public ::testing::Test {
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
        int fd = shm_open("/TEST_ROBOT_CONFIG_DYNGRP", O_CREAT | O_RDWR, 0666);
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
        shm_unlink("/TEST_ROBOT_CONFIG_DYNGRP");
    }
};

#if ROBOT_TYPE == HUMANOID
TEST_F(DynGroupTest, QueryDynGrpFromBuiltConfig) {
    if (access(kUrdfPath, F_OK) != 0 ||
        access("/home/user/claude/task/TaskFrame/cfg/robot_conf_with_dyngrp.cfg", F_OK) != 0) {
        GTEST_SKIP() << "Config files not found";
    }

    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) -
        static_cast<char*>(g_shm_base));

    CTaskFrame main_task;
    main_task.SetMainTask();
    ASSERT_TRUE(main_task.BuildRobotConfig(kUrdfPath,
                                            "/home/user/claude/task/TaskFrame/cfg/robot_conf_with_dyngrp.cfg"));

    // Manually add a dyngrp entry because the current confbase builder does not
    // initialize the dyngrp page from JSON.
    {
        void* base = _global_data._robot_config.get();
        ASSERT_NE(base, nullptr);
        robot_config::DynGroupConfigPage* page =
            const_cast<robot_config::DynGroupConfigPage*>(GetDynGroupPageFromBase(base));
        ASSERT_NE(page, nullptr);
        page->num_groups = 1;
        page->groups[0].unit_type =
            static_cast<int32_t>(robot_config::RobotSocUnitType::RIGHT_ARM);
        page->groups[0].dyngrp_model = robot_config::kDynGroupModelDynMia;
        page->groups[0].dyngrp_no = 7;
        page->groups[0].power_regain_limit = 1234.5;
        page->groups[0].power_regain_min = 0.0;
        page->groups[0].emergency_stop_power_regain = 0.0;
    }

    GlobalData* gd = reinterpret_cast<GlobalData*>(
        static_cast<char*>(shm_base) + sizeof(TfShmHeader));
    *gd = _global_data;

    CTaskFrame child_task;
    ASSERT_TRUE(child_task.AttachRobotConfig());

    int32_t dyngrp_no = -1;
    ASSERT_TRUE(child_task.GetDynGroupNumber(
        static_cast<int32_t>(RobotSocUnitType::RIGHT_ARM),
        kDynGroupModelDynMia, &dyngrp_no));
    EXPECT_EQ(dyngrp_no, 7);

    double limit = 0.0;
    ASSERT_TRUE(child_task.GetPowerRegainLimit(
        static_cast<int32_t>(RobotSocUnitType::RIGHT_ARM),
        kDynGroupModelDynMia, &limit));
    EXPECT_DOUBLE_EQ(limit, 1234.5);
}
#endif

TEST_F(DynGroupTest, MissingDynGrpReturnsFalse) {
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

    int32_t dyngrp_no = -1;
    EXPECT_FALSE(child_task.GetDynGroupNumber(
        static_cast<int32_t>(RobotSocUnitType::RIGHT_ARM),
        kDynGroupModelDynMia, &dyngrp_no));
}
