/**
 * @file full_app_state_machine_test.cpp
 * @brief task/app 全应用状态机切换集成测试（需要真实 EtherCAT 从站在线，手动运行）
 *
 * 驱动：HTTP HMI  POST 127.0.0.1:8080/stateMsg
 * 观测：attach /dev/shm/ROBOT_GLOBAL_DATA 读 StatusManagerData（4 进程状态字）
 * 生命周期：本测试 fork/exec 托管 taskframe_launcher
 */

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <string>

#include "httplib.h"

#include "shm_header.hpp"
#include "shm_ptr.hpp"
#include "status_manager.hpp"
#include "task_frame.hpp"   // GlobalData / SHARED_MEMORY_NAME

namespace {

constexpr char kAppDir[] = "/home/user/claude/task/app";
constexpr int kHmiPort = 8080;
constexpr int kWorkerCount = 4;
constexpr const char* kWorkerElfNames[kWorkerCount] = {
    "state_master.elf", "planning_task.elf", "servo_task.elf", "fieldbus_task.elf"};
constexpr const char* kTaskNames[kWorkerCount] = {
    "state_master", "planning_task", "servo_task", "fieldbus_task"};

// 扫 /proc 判断进程是否存在（避免 pgrep 自匹配），与 test_four_process.cpp 同款
bool process_exists(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return false;
    bool found = false;
    pid_t self = getpid();
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (!isdigit(entry->d_name[0])) continue;
        pid_t pid = atoi(entry->d_name);
        if (pid == self) continue;
        char path[256];
        snprintf(path, sizeof(path), "/proc/%d/cmdline", pid);
        FILE* fp = fopen(path, "r");
        if (!fp) continue;
        char cmdline[256];
        size_t n = fread(cmdline, 1, sizeof(cmdline) - 1, fp);
        fclose(fp);
        if (n == 0) continue;
        cmdline[n] = '\0';
        if (strstr(cmdline, name) != nullptr) { found = true; break; }
    }
    closedir(dir);
    return found;
}

void kill_process(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return;
    pid_t self = getpid();
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (!isdigit(entry->d_name[0]) || (pid_t)atoi(entry->d_name) == self) continue;
        char path[256];
        snprintf(path, sizeof(path), "/proc/%d/cmdline", atoi(entry->d_name));
        FILE* fp = fopen(path, "r");
        if (!fp) continue;
        char cmdline[256];
        size_t n = fread(cmdline, 1, sizeof(cmdline) - 1, fp);
        fclose(fp);
        if (n == 0) continue;
        cmdline[n] = '\0';
        if (strstr(cmdline, name) != nullptr) kill(atoi(entry->d_name), SIGKILL);
    }
    closedir(dir);
}

bool PortInUse(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    bool in_use = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    return in_use;
}

// fork/exec 托管 taskframe_launcher；返回 launcher pid，失败返回 -1。
// stdin 必须接 /dev/null（EOF）：launcher 主循环 getchar() 收到字符才退出、
// EOF 继续循环；若接保持打开的空 pipe，getchar() 会永久阻塞，导致 SIGTERM
// 后 _running 标志无人检查、优雅退出失效。
pid_t LauncherStart() {
    shm_unlink(SHARED_MEMORY_NAME);
    kill_process("taskframe_launcher");
    for (const char* elf : kWorkerElfNames) kill_process(elf);
    usleep(200000);

    pid_t pid = fork();
    if (pid == 0) {
        int devnull = open("/dev/null", O_RDONLY);
        if (devnull >= 0) {
            dup2(devnull, STDIN_FILENO);
            close(devnull);
        }
        chdir(kAppDir);
        execl("./taskframe_launcher", "taskframe_launcher", nullptr);
        _exit(127);
    }
    return pid;
}

void LauncherStop(pid_t pid) {
    if (pid > 0) {
        kill(pid, SIGTERM);
        int status = 0;
        int waited = 0;
        // launcher 优雅退出最坏 ~10s（每 worker 最多 5s 的 SIGKILL 兜底）
        while (waited < 15000) {
            if (waitpid(pid, &status, WNOHANG) == pid) break;
            usleep(10000);
            waited += 10;
        }
        if (waited >= 15000) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
        }
    }
    // 兜底：确保无残留（正常路径 launcher 已负责）
    for (const char* elf : kWorkerElfNames) kill_process(elf);
    shm_unlink(SHARED_MEMORY_NAME);
}

// 只读 attach /ROBOT_GLOBAL_DATA，遍历 StatusManagerData 状态表。
// 全部为纯数据 struct 字段访问，无需链接 libtaskframe。
class ShmStatusWatcher {
public:
    ~ShmStatusWatcher() { Detach(); }

    // 幂等；shm 尚未创建或头未就绪时返回 false（由调用方轮询）
    bool Attach() {
        if (base_ != nullptr) return true;
        int fd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0);
        if (fd < 0) return false;
        ShmHeader header{};
        ssize_t n = pread(fd, &header, sizeof(header), 0);
        if (n != (ssize_t)sizeof(header) || header.magic != SHM_MAGIC || header.size == 0) {
            close(fd);
            return false;
        }
        size_ = header.size;
        void* base = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (base == MAP_FAILED) return false;
        base_ = base;
        global_ = reinterpret_cast<GlobalData*>(static_cast<char*>(base_) + sizeof(ShmHeader));
        return true;
    }

    void Detach() {
        if (base_ != nullptr) { munmap(base_, size_); base_ = nullptr; }
    }

    // 状态表就绪：_task_status 偏移非 0 且 4 个任务名全部注册
    bool StatusTableReady() {
        if (!Attach()) return false;
        size_t off = global_->_task_status.offset();
        if (off == 0) return false;
        StatusManagerData* smd = StatusData();
        if (smd->magic != STATUS_MANAGER_MAGIC) return false;
        for (const char* name : kTaskNames) {
            int dummy;
            if (!GetStatusByName(name, &dummy)) return false;
        }
        return true;
    }

    bool GetStatusByName(const char* name, int* out) {
        StatusManagerData* smd = StatusData();
        if (smd == nullptr) return false;
        int count = smd->task_number;
        if (count > MAX_TASK_NUMBER) count = MAX_TASK_NUMBER;
        for (int i = 0; i < count; i++) {
            if (strncmp(name, smd->task_status[i]._task_name, 32) == 0) {
                *out = smd->task_status[i]._status_value.load();
                return true;
            }
        }
        return false;
    }

    // 轮询直到 4 个任务全部到达 expected；超时返回 false（用 Snapshot() 看现场）
    bool WaitForAllTasks(int expected, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            bool all = true;
            for (const char* name : kTaskNames) {
                int v = -1;
                if (!GetStatusByName(name, &v) || v != expected) { all = false; break; }
            }
            if (all) return true;
            usleep(20000);
        }
        return false;
    }

    std::string Snapshot() {
        std::string s = "status table snapshot:\n";
        StatusManagerData* smd = StatusData();
        if (smd == nullptr) return s + "  (shm not attached)\n";
        char line[128];
        for (const char* name : kTaskNames) {
            int v = -1;
            bool ok = GetStatusByName(name, &v);
            snprintf(line, sizeof(line), "  %-16s %s\n", name,
                     ok ? std::to_string(v).c_str() : "(not registered)");
            s += line;
        }
        return s;
    }

private:
    StatusManagerData* StatusData() {
        if (base_ == nullptr) return nullptr;
        size_t off = global_->_task_status.offset();
        if (off == 0) return nullptr;
        return reinterpret_cast<StatusManagerData*>(static_cast<char*>(base_) + off);
    }

    void* base_ = nullptr;
    size_t size_ = 0;
    GlobalData* global_ = nullptr;
};

class HmiClient {
public:
    // 成功（200 + "OK!"）仅代表事件已入 StateMaster 自身事件环，不代表切换完成
    bool SendFsmCommand(const char* state_msg, std::string* err) {
        httplib::Client cli("127.0.0.1", kHmiPort);
        cli.set_connection_timeout(2, 0);
        cli.set_read_timeout(5, 0);
        std::string body = std::string(R"({"cmd":"REFSTATECHANGE","stateMSG":")")
                           + state_msg + "\"}";
        auto res = cli.Post("/stateMsg", body, "application/json");
        if (!res) { *err = "no response (connection failed)"; return false; }
        if (res->status != 200 || res->body.find("OK!") == std::string::npos) {
            *err = "HTTP " + std::to_string(res->status) + ": " + res->body;
            return false;
        }
        return true;
    }
};

}  // namespace

// 环境前置自检：不启动应用，只验证测试前提，无硬件也应通过
TEST(FullAppEnvTest, DeployedArtifactsExist) {
    for (const char* elf : kWorkerElfNames) {
        std::string path = std::string(kAppDir) + "/" + elf;
        EXPECT_EQ(access(path.c_str(), X_OK), 0) << "missing executable: " << path;
    }
    std::string launcher = std::string(kAppDir) + "/taskframe_launcher";
    EXPECT_EQ(access(launcher.c_str(), X_OK), 0) << "missing: " << launcher;
    std::string yaml = std::string(kAppDir) + "/cfg/taskframe_launcher.yaml";
    EXPECT_EQ(access(yaml.c_str(), R_OK), 0) << "missing: " << yaml;
}

TEST(FullAppEnvTest, HmiPortFree) {
    EXPECT_FALSE(PortInUse(kHmiPort))
        << "port 8080 already in use — is another state_master.elf running?";
}

TEST(FullAppLaunchTest, LauncherStartsAndWorkersRegister) {
    ASSERT_FALSE(PortInUse(kHmiPort)) << "port 8080 occupied";

    
    pid_t launcher = LauncherStart();
    ASSERT_GT(launcher, 0) << "fork failed";

    // 等待 4 个 worker 进程全部出现（Launcher 先起 state_master，等 RobotConfig 后再起其余 3 个）
    bool all_up = false;
    for (int waited = 0; waited < 20000 && !all_up; waited += 50) {
        all_up = true;
        for (const char* elf : kWorkerElfNames) {
            if (!process_exists(elf)) { all_up = false; break; }
        }
        if (!all_up) usleep(50000);
    }
    EXPECT_TRUE(all_up) << "4 workers did not start within 20s";

    LauncherStop(launcher);

    for (const char* elf : kWorkerElfNames) {
        EXPECT_FALSE(process_exists(elf)) << elf << " survived launcher shutdown";
    }
}

TEST(FullAppLaunchTest, AllTasksStartUninitiated) {
    ASSERT_FALSE(PortInUse(kHmiPort)) << "port 8080 occupied";

    
    pid_t launcher = LauncherStart();
    ASSERT_GT(launcher, 0);

    ShmStatusWatcher watcher;
    bool ready = false;
    for (int waited = 0; waited < 20000 && !ready; waited += 50) {
        ready = watcher.StatusTableReady();
        if (!ready) usleep(50000);
    }
    ASSERT_TRUE(ready) << "status table not ready within 20s\n" << watcher.Snapshot();

    // 启动后、未发任何事件：4 进程都应停在 UNINTIATED
    EXPECT_TRUE(watcher.WaitForAllTasks(UNINTIATED, 5000)) << watcher.Snapshot();

    watcher.Detach();
    LauncherStop(launcher);
}

class FullAppStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_FALSE(PortInUse(kHmiPort)) << "port 8080 occupied";
        launcher_pid_ = LauncherStart();
        ASSERT_GT(launcher_pid_, 0) << "fork failed";
        bool ready = false;
        for (int waited = 0; waited < 20000 && !ready; waited += 50) {
            ready = watcher_.StatusTableReady();
            if (!ready) usleep(50000);
        }
        ASSERT_TRUE(ready) << "app not ready within 20s\n" << watcher_.Snapshot();
        ASSERT_TRUE(watcher_.WaitForAllTasks(UNINTIATED, 5000)) << watcher_.Snapshot();
    }

    void TearDown() override {
        watcher_.Detach();
        LauncherStop(launcher_pid_);
    }

    // 下发命令并等待 4 进程全部到达 expected。
    // timeout 默认 10s，必须大于主任务 80 周期≈5.1s 的同步窗口。
    void Step(const char* msg, int expected, int timeout_ms = 10000) {
        std::string err;
        ASSERT_TRUE(hmi_.SendFsmCommand(msg, &err))
            << msg << " rejected by HMI: " << err;
        ASSERT_TRUE(watcher_.WaitForAllTasks(expected, timeout_ms))
            << "after " << msg << ":\n" << watcher_.Snapshot();
    }

    pid_t launcher_pid_ = -1;
    ShmStatusWatcher watcher_;
    HmiClient hmi_;
};

TEST_F(FullAppStateMachineTest, InitTransition) {
    // INIT 含 Fieldbus 真实 EtherCAT 冷启动（主站激活+等 OP+SDO 读伺服），
    // 主任务为此切换加宽同步窗口到 300 周期≈19.2s，此处留足余量
    Step("msg_initialize", UNCONFIGED, 25000);
    Step("msg_error", ERROR);        // 收尾回可重启状态
    Step("msg_reset_ok", MOTOR_OFF);
}

TEST_F(FullAppStateMachineTest, Estop_FromMotion) {
    Step("msg_initialize",  UNCONFIGED, 25000);  // INIT：含真实 EtherCAT 冷启动
    Step("msg_config",      MOTOR_OFF);          // CONFIG
    Step("msg_motor_on",    READY, 15000);       // MOTOR_ON：真实伺服使能
    Step("msg_control_on",  RUNNING);            // CONTROL_ON
    Step("msg_estop",       EMERGENCY_STOP);     // 急停：断主电、抱闸抱紧
    Step("msg_reset_ok",    MOTOR_OFF);          // 人工复位，保证可重复运行
}

TEST_F(FullAppStateMachineTest, GoldenPath_InitToError) {
    // SetUp 已确认全部 UNINTIATED
    Step("msg_initialize",  UNCONFIGED, 25000);  // INIT：含真实 EtherCAT 冷启动
    Step("msg_config",      MOTOR_OFF);          // CONFIG
    Step("msg_motor_on",    READY, 15000);       // MOTOR_ON：真实伺服使能
    Step("msg_control_on",  RUNNING);            // CONTROL_ON
    Step("msg_control_off", READY);              // CONTROL_OFF
    Step("msg_motor_off",   MOTOR_OFF);          // MOTOR_OFF
    Step("msg_error",       ERROR);              // 显式触发 ERROR
    Step("msg_reset_ok",    MOTOR_OFF);          // 收尾：RESET 回 MOTOR_OFF，保证可重复运行
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}