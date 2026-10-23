// Launcher YAML 配置解析的表征测试。
#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "config_parser.hpp"

namespace {

class ConfigParserTest : public ::testing::Test {
 protected:
    void TearDown() override {
        for (const auto& path : paths_) unlink(path.c_str());
    }

    std::string WriteYaml(const std::string& content) {
        std::string path = "config_parser_test_" + std::to_string(getpid()) +
                           "_" + std::to_string(paths_.size()) + ".yaml";
        std::ofstream ofs(path);
        ofs << content;
        ofs.close();
        paths_.push_back(path);
        return path;
    }

 private:
    std::vector<std::string> paths_;
};

TEST_F(ConfigParserTest, ParsesFullConfig) {
    std::string path = WriteYaml(R"(
system:
  shm_name: "/TEST_SHM"
  shm_size: 1048576
  watchdog_interval_ms: 50
  heartbeat_timeout_ms: 200
tasks:
  - name: "state_master"
    executable: "./state_master.elf"
    priority: 80
    cpu: 2
    autorestart: true
    max_restarts: 3
  - name: "servo_task"
    executable: "./servo_task.elf"
    priority: 96
    cpu: 3
    autorestart: false
    max_restarts: 0
)");
    LauncherConfig config;
    ASSERT_TRUE(parse_config(path, config));

    EXPECT_EQ(config.system.shm_name, "/TEST_SHM");
    EXPECT_EQ(config.system.shm_size, 1048576u);
    EXPECT_EQ(config.system.watchdog_interval_ms, 50);
    EXPECT_EQ(config.system.heartbeat_timeout_ms, 200);

    ASSERT_EQ(config.workers.size(), 2u);
    EXPECT_EQ(config.workers[0].name, "state_master");
    EXPECT_EQ(config.workers[0].executable, "./state_master.elf");
    EXPECT_EQ(config.workers[0].priority, 80);
    EXPECT_EQ(config.workers[0].cpu, 2);
    EXPECT_TRUE(config.workers[0].autorestart);
    EXPECT_EQ(config.workers[0].max_restarts, 3);
    EXPECT_EQ(config.workers[1].name, "servo_task");
    EXPECT_FALSE(config.workers[1].autorestart);
    EXPECT_EQ(config.workers[1].max_restarts, 0);
}

TEST_F(ConfigParserTest, MissingFileReturnsFalse) {
    LauncherConfig config;
    EXPECT_FALSE(parse_config("no_such_file_12345.yaml", config));
}

TEST_F(ConfigParserTest, MalformedYamlReturnsFalse) {
    std::string path = WriteYaml("system: [unclosed\n  : : :");
    LauncherConfig config;
    EXPECT_FALSE(parse_config(path, config));
}

TEST_F(ConfigParserTest, OptionalTaskFieldsKeepDefaults) {
    std::string path = WriteYaml(R"(
tasks:
  - name: "minimal"
    executable: "./minimal.elf"
)");
    LauncherConfig config;
    ASSERT_TRUE(parse_config(path, config));

    ASSERT_EQ(config.workers.size(), 1u);
    EXPECT_EQ(config.workers[0].priority, 80);
    EXPECT_EQ(config.workers[0].cpu, 0);
    EXPECT_TRUE(config.workers[0].autorestart);
    EXPECT_EQ(config.workers[0].max_restarts, 5);
    // system 段缺省时保留 SystemConfig 默认值
    EXPECT_EQ(config.system.shm_name, "/ROBOT_GLOBAL_DATA");
}

TEST(DefaultConfigTest, HasFourWorkersWithStateMasterFirst) {
    LauncherConfig config;
    load_default_config(config);

    ASSERT_EQ(config.workers.size(), 4u);
    EXPECT_EQ(config.workers[0].name, "state_master");
    EXPECT_EQ(config.workers[1].name, "planning_task");
    EXPECT_EQ(config.workers[2].name, "servo_task");
    EXPECT_EQ(config.workers[3].name, "fieldbus_task");
    for (const auto& w : config.workers) {
        EXPECT_TRUE(w.autorestart) << w.name;
        EXPECT_EQ(w.max_restarts, 3) << w.name;
    }
}

TEST(DefaultConfigTest, SystemDefaults) {
    LauncherConfig config;
    load_default_config(config);

    EXPECT_EQ(config.system.shm_name, "/ROBOT_GLOBAL_DATA");
    EXPECT_EQ(config.system.shm_size, 100u * 1024 * 1024);
    EXPECT_EQ(config.system.watchdog_interval_ms, 100);
    EXPECT_EQ(config.system.heartbeat_timeout_ms, 500);
}

}  // namespace
