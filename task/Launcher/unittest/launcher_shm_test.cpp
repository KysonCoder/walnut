// Launcher 残留共享内存清理的单元测试。
// 覆盖：CleanupStaleShm 的各分支，以及 start_all_workers 在启动 worker
// 之前清理残留 shm 的接线（root 残留导致 Permission denied 的回归防护）。
#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

#include "launcher.hpp"

namespace {

std::string ShmFilePath(const std::string& shm_name) {
    return "/dev/shm/" +
        (shm_name[0] == '/' ? shm_name.substr(1) : shm_name);
}

// 直接查 /dev/shm 下的文件名——shm_open 在文件无权限时也会失败，
// 不能用来判定"是否还存在"。
bool ShmFileExists(const std::string& shm_name) {
    return access(ShmFilePath(shm_name).c_str(), F_OK) == 0;
}

class LauncherShmTest : public ::testing::Test {
 protected:
    void TearDown() override {
        for (const auto& name : shm_names_) shm_unlink(name.c_str());
        for (const auto& path : cfg_paths_) unlink(path.c_str());
        for (const auto& path : dir_paths_) rmdir(path.c_str());
    }

    // 每个用例使用独立 shm 名（含 pid），避免与其他测试/残留互相干扰。
    std::string MakeShmName(const char* suffix) {
        shm_names_.push_back("/launcher_test_" + std::string(suffix) + "_" +
                             std::to_string(getpid()));
        return shm_names_.back();
    }

    void CreateShm(const std::string& name, mode_t mode) {
        int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(ftruncate(fd, 4096), 0);
        if (mode != 0600) ASSERT_EQ(fchmod(fd, mode), 0);
        close(fd);
    }

    // 写一份只含 system.shm_name、无 tasks 的最小配置，返回路径。
    std::string WriteConfig(const std::string& shm_name) {
        std::string path = "launcher_test_cfg_" + std::to_string(getpid()) +
                           "_" + std::to_string(cfg_paths_.size()) + ".yaml";
        std::ofstream ofs(path);
        ofs << "system:\n  shm_name: \"" << shm_name << "\"\n";
        ofs.close();
        cfg_paths_.push_back(path);
        return path;
    }

    // 在 /dev/shm 下建同名目录，制造"存在但无法 shm_unlink"的冲突
    // （非 root 无法模拟 root 拥有的文件，这是等价的失败路径）。
    void CreateBlockingDir(const std::string& shm_name) {
        std::string path = ShmFilePath(shm_name);
        ASSERT_EQ(mkdir(path.c_str(), 0700), 0);
        dir_paths_.push_back(path);
    }

 private:
    std::vector<std::string> shm_names_;
    std::vector<std::string> cfg_paths_;
    std::vector<std::string> dir_paths_;
};

TEST_F(LauncherShmTest, CleanupReturnsTrueWhenNoStaleShm) {
    EXPECT_TRUE(Launcher::CleanupStaleShm(MakeShmName("absent")));
}

TEST_F(LauncherShmTest, CleanupRemovesStaleShm) {
    std::string name = MakeShmName("stale");
    CreateShm(name, 0600);
    ASSERT_TRUE(ShmFileExists(name));

    EXPECT_TRUE(Launcher::CleanupStaleShm(name));
    EXPECT_FALSE(ShmFileExists(name));
}

TEST_F(LauncherShmTest, CleanupRemovesUnreadableStaleShm) {
    // 模拟 root 运行残留：文件无读写权限，shm_open(O_RDWR) 得到 EACCES。
    // /dev/shm 是 sticky 目录，owner 仍可 unlink——非 root 重跑应能自救。
    std::string name = MakeShmName("eacces");
    CreateShm(name, 0000);
    ASSERT_TRUE(ShmFileExists(name));

    EXPECT_TRUE(Launcher::CleanupStaleShm(name));
    EXPECT_FALSE(ShmFileExists(name));
}

TEST_F(LauncherShmTest, CleanupFailsWhenNameIsNotRemovable) {
    // 同名目录：shm_open 失败（非 ENOENT）且 shm_unlink 不掉目录 → false。
    // 与 root 残留（EPERM）殊途同归，覆盖"清理失败"分支。
    std::string name = MakeShmName("blocked");
    CreateBlockingDir(name);

    EXPECT_FALSE(Launcher::CleanupStaleShm(name));
    EXPECT_TRUE(ShmFileExists(name));  // 未被误删
}

TEST_F(LauncherShmTest, StartAllWorkersCleansStaleShmBeforeStart) {
    std::string shm_name = MakeShmName("wiring");
    CreateShm(shm_name, 0600);
    ASSERT_TRUE(ShmFileExists(shm_name));

    Launcher launcher;
    ASSERT_TRUE(launcher.init(WriteConfig(shm_name)));

    // 配置里没有任何 worker：start_all_workers 只应做清理并立即成功。
    EXPECT_TRUE(launcher.start_all_workers());
    EXPECT_FALSE(ShmFileExists(shm_name));
}

TEST_F(LauncherShmTest, StartAllWorkersFailsWhenStaleShmNotRemovable) {
    std::string shm_name = MakeShmName("blockedstart");
    CreateBlockingDir(shm_name);

    Launcher launcher;
    ASSERT_TRUE(launcher.init(WriteConfig(shm_name)));

    EXPECT_FALSE(launcher.start_all_workers());
}

TEST_F(LauncherShmTest, StopAllWorkersWithoutShmDoesNotHang) {
    // shm 从未创建过时，stop_all_workers（含 graceful_motor_off）应立即返回。
    Launcher launcher;
    ASSERT_TRUE(launcher.init(WriteConfig(MakeShmName("nostop"))));
    launcher.stop_all_workers();
    SUCCEED();
}

}  // namespace
