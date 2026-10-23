// WorkerManager 的表征测试，使用两个真实的最小 worker ELF：
//   launcher_test_worker.elf        —— 注册 + 周期心跳，收 LAUNCHER_STOP 退出
//   launcher_test_silent_worker.elf —— 注册后沉默（触发心跳超时路径）
// 运行目录需为 output/（ctest WORKING_DIRECTORY 已设置），以便 "./xxx.elf" 可解析。
#include <gtest/gtest.h>

#include <errno.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <string>

#include "ipc_protocol.hpp"
#include "worker_manager.hpp"

namespace {

constexpr const char* kWorkerElf = "./launcher_test_worker.elf";
constexpr const char* kSilentWorkerElf = "./launcher_test_silent_worker.elf";

WorkerConfig MakeWorker(const std::string& name, const std::string& exe,
                        bool autorestart, int max_restarts) {
    WorkerConfig wc;
    wc.name = name;
    wc.executable = exe;
    wc.autorestart = autorestart;
    wc.max_restarts = max_restarts;
    return wc;
}

}  // namespace

TEST(WorkerManagerTest, StartAllSpawnsWorkerWhichRegisters) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_reg", kWorkerElf, false, 0));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    ASSERT_EQ(mgr.workers().size(), 1u);
    EXPECT_TRUE(mgr.workers()[0].running);
    EXPECT_GT(mgr.workers()[0].pid, 0);
    EXPECT_GE(mgr.workers()[0].ctrl_socket, 0);

    ipc::CtrlMessage msg;
    ASSERT_EQ(ipc::recv_ctrl_message(mgr.workers()[0].ctrl_socket, msg, 2000),
              static_cast<ssize_t>(sizeof(msg)));
    EXPECT_EQ(msg.type, ipc::CtrlMsgType::WORKER_REGISTER);
    EXPECT_EQ(msg.pid, mgr.workers()[0].pid);
    EXPECT_STREQ(msg.task_name, "w_reg");

    mgr.stop_all();
    EXPECT_FALSE(mgr.workers()[0].running);
}

TEST(WorkerManagerTest, StopAllTerminatesAndReapsChild) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_stop", kWorkerElf, false, 0));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    pid_t pid = mgr.workers()[0].pid;

    mgr.stop_all();
    EXPECT_FALSE(mgr.workers()[0].running);
    // 子进程已被回收：kill(pid, 0) 应报 ESRCH
    EXPECT_EQ(kill(pid, 0), -1);
    EXPECT_EQ(errno, ESRCH);
}

TEST(WorkerManagerTest, HandleHeartbeatRefreshesTimestamp) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_hb", kWorkerElf, false, 0));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    auto before = mgr.workers()[0].last_heartbeat;
    usleep(20000);

    mgr.handle_heartbeat(mgr.workers()[0].pid);
    EXPECT_GT(mgr.workers()[0].last_heartbeat, before);

    mgr.stop_all();
}

TEST(WorkerManagerTest, HandleWorkerExitMarksNotRunning) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_exit", "/bin/true", false, 0));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    pid_t pid = mgr.workers()[0].pid;

    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);  // /bin/true 立即退出
    mgr.handle_worker_exit(pid, status);
    EXPECT_FALSE(mgr.workers()[0].running);
}

TEST(WorkerManagerTest, HeartbeatTimeoutKillsAndRestartsWorker) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_silent", kSilentWorkerElf,
                                        /*autorestart=*/true, /*max_restarts=*/3));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    pid_t first_pid = mgr.workers()[0].pid;
    ASSERT_EQ(mgr.workers()[0].restart_count, 1);

    usleep(700000);  // 超过 500ms 心跳超时
    mgr.check_and_restart();

    EXPECT_TRUE(mgr.workers()[0].running);
    EXPECT_NE(mgr.workers()[0].pid, first_pid);
    EXPECT_EQ(mgr.workers()[0].restart_count, 2);
    EXPECT_EQ(kill(first_pid, 0), -1);  // 旧进程已被 SIGKILL + 回收

    mgr.stop_all();
}

TEST(WorkerManagerTest, NoRestartWhenAutorestartDisabled) {
    LauncherConfig config;
    config.workers.push_back(MakeWorker("w_norestart", kSilentWorkerElf,
                                        /*autorestart=*/false, /*max_restarts=*/0));

    WorkerManager mgr;
    ASSERT_TRUE(mgr.start_all(config));
    usleep(700000);
    mgr.check_and_restart();

    EXPECT_FALSE(mgr.workers()[0].running);

    mgr.stop_all();
}

TEST(WorkerManagerTest, StartAllFailsOnMissingExecutable) {
    LauncherConfig config;
    config.workers.push_back(
        MakeWorker("w_missing", "./no_such_worker.elf", false, 0));

    WorkerManager mgr;
    // execv 失败的子进程 _exit(1)；start_worker 本身（fork 成功）仍返回 true，
    // 但 worker 会立刻退出——验证 manager 能观察到退出而不是卡死。
    ASSERT_TRUE(mgr.start_all(config));
    pid_t pid = mgr.workers()[0].pid;
    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 1);
    mgr.handle_worker_exit(pid, status);
    EXPECT_FALSE(mgr.workers()[0].running);

    mgr.stop_all();
}

