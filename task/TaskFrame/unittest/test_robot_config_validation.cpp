#include "task_frame.hpp"
#include "confbase/robot.hpp"
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <string>

#if ROBOT_TYPE == HUMANOID
constexpr const char* kUrdfPath = "/home/user/claude/cfg/humanoid.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/humanoid_robot_conf.cfg";
#else
constexpr const char* kUrdfPath = "/home/user/claude/cfg/2arm_v1.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/dualarm_robot_conf.cfg";
#endif

extern GlobalData _global_data;

class RobotConfigValidationTest : public ::testing::Test {
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
        int fd = shm_open("/TEST_ROBOT_CONFIG_VALID", O_CREAT | O_RDWR, 0666);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(ftruncate(fd, kTestShmSize), 0);
        shm_base = mmap(nullptr, kTestShmSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
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
        robot_config::RobotConfigStore::Instance().Detach();
        if (shm_base) munmap(shm_base, kTestShmSize);
        shm_unlink("/TEST_ROBOT_CONFIG_VALID");
    }

    std::string WriteTempCfg(const std::string& content) {
        std::string path = "/tmp/rc_validation_test_" + std::to_string(getpid()) + ".cfg";
        std::ofstream ofs(path);
        ofs << content;
        return path;
    }
};

#if ROBOT_TYPE == HUMANOID
TEST_F(RobotConfigValidationTest, MissingKinematicModelFails) {
    if (access(kUrdfPath, F_OK) != 0) {
        GTEST_SKIP() << "URDF not found";
    }

    std::string cfg = R"({
        "ratio": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "acceleration": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "Deceleration": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "zeroPosition": {"rightArm":[0,0,0,0,0,0,0],"leftArm":[0,0,0,0,0,0,0]},
        "encoderResolutionCounter": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "servoID": {"rightArm":[-1,-1,-1,-1,-1,-1,-1],"leftArm":[-1,-1,-1,-1,-1,-1,-1]},
        "root_link": {"rightArm":"Body","leftArm":"Body"},
        "tip_link": {"rightArm":"Rarm_ter","leftArm":"Larm_ter"},
        "robot_topology": {"global_base_link":"Base","chains":[{"name":"rightArm","type":"arm","units":["rightArm"]},{"name":"leftArm","type":"arm","units":["leftArm"]}]}
    })";

    std::string cfg_path = WriteTempCfg(cfg);

    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) -
        static_cast<char*>(g_shm_base));

    CTaskFrame task;
    task.SetMainTask();
    EXPECT_FALSE(task.BuildRobotConfig(kUrdfPath, cfg_path.c_str()))
        << "BuildRobotConfig should fail when kinematic_model is missing";

    std::remove(cfg_path.c_str());
}

TEST_F(RobotConfigValidationTest, MissingRobotTopologyFails) {
    if (access(kUrdfPath, F_OK) != 0) {
        GTEST_SKIP() << "URDF not found";
    }

    std::string cfg = R"({
        "ratio": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "acceleration": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "Deceleration": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "zeroPosition": {"rightArm":[0,0,0,0,0,0,0],"leftArm":[0,0,0,0,0,0,0]},
        "encoderResolutionCounter": {"rightArm":[1,1,1,1,1,1,1],"leftArm":[1,1,1,1,1,1,1]},
        "servoID": {"rightArm":[-1,-1,-1,-1,-1,-1,-1],"leftArm":[-1,-1,-1,-1,-1,-1,-1]},
        "kinematic_model": {"rightArm":"CKinGeneRbt","leftArm":"CKinGeneRbt"},
        "root_link": {"rightArm":"Body","leftArm":"Body"},
        "tip_link": {"rightArm":"Rarm_ter","leftArm":"Larm_ter"}
    })";

    std::string cfg_path = WriteTempCfg(cfg);

    _global_data._robot_config = ShmPtr<char>(
        static_cast<char*>(_global_data._available_data_ptr) -
        static_cast<char*>(g_shm_base));

    CTaskFrame task;
    task.SetMainTask();
    EXPECT_FALSE(task.BuildRobotConfig(kUrdfPath, cfg_path.c_str()))
        << "BuildRobotConfig should fail when robot_topology is missing";

    std::remove(cfg_path.c_str());
}
#endif
