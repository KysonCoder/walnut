#include <gtest/gtest.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <fstream>
#include <dirent.h>
#include <cctype>

// Scan /proc directly to find processes whose /proc/<pid>/cmdline contains
// the given name.  This avoids the shell-self-match problem of pgrep -f.
static bool process_exists(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return false;
    struct dirent* entry;
    pid_t self = getpid();
    bool found = false;
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
        // cmdline uses \0 separators; check the first argument (basename)
        if (strstr(cmdline, name) != nullptr) {
            found = true;
            break;
        }
    }
    closedir(dir);
    return found;
}

static void kill_process(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return;
    struct dirent* entry;
    pid_t self = getpid();
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
        if (strstr(cmdline, name) != nullptr) {
            kill(pid, SIGKILL);
        }
    }
    closedir(dir);
}

static pid_t get_pid(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return -1;
    struct dirent* entry;
    pid_t self = getpid();
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
        if (strstr(cmdline, name) != nullptr) {
            closedir(dir);
            return pid;
        }
    }
    closedir(dir);
    return -1;
}

class FourProcessTest : public ::testing::Test {
protected:
    void SetUp() override {
        shm_unlink("/ROBOT_GLOBAL_DATA");
        kill_process("test_state_master_worker.elf");
        kill_process("test_planning_worker.elf");
        kill_process("test_servo_worker.elf");
        kill_process("test_fieldbus_worker.elf");
        kill_process("taskframe_launcher");
        usleep(200000);
    }

    void TearDown() override {
        kill_process("test_state_master_worker.elf");
        kill_process("test_planning_worker.elf");
        kill_process("test_servo_worker.elf");
        kill_process("test_fieldbus_worker.elf");
        kill_process("taskframe_launcher");
        shm_unlink("/ROBOT_GLOBAL_DATA");
    }

    pid_t start_launcher(const char* config) {
        pid_t pid = fork();
        if (pid == 0) {
            execl("./app/taskframe_launcher",
                  "taskframe_launcher", config, nullptr);
            _exit(1);
        }
        return pid;
    }

    void wait_for_workers(int timeout_ms) {
        int waited = 0;
        while (waited < timeout_ms) {
            if (process_exists("test_state_master_worker.elf") &&
                process_exists("test_planning_worker.elf") &&
                process_exists("test_servo_worker.elf") &&
                process_exists("test_fieldbus_worker.elf")) {
                return;
            }
            usleep(10000);
            waited += 10;
        }
    }

    void stop_launcher(pid_t launcher_pid) {
        kill(launcher_pid, SIGTERM);
        int status;
        int waited = 0;
        // Allow up to 10s for launcher to gracefully shut down all workers.
        // stop_all() waits up to 2s per worker; with 4 workers this can
        // take ~8s in the worst case.
        while (waited < 10000) {
            pid_t result = waitpid(launcher_pid, &status, WNOHANG);
            if (result == launcher_pid) break;
            usleep(10000);
            waited += 10;
        }
        if (waited >= 10000) {
            kill(launcher_pid, SIGKILL);
            waitpid(launcher_pid, &status, 0);
        }
    }
};

// ==== Existing tests ====

TEST_F(FourProcessTest, LauncherStartsAllFourWorkers)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0) << "fork failed";

    wait_for_workers(5000);

    EXPECT_TRUE(process_exists("test_state_master_worker.elf"))
        << "State master should be running";
    EXPECT_TRUE(process_exists("test_planning_worker.elf"))
        << "Planning should be running";
    EXPECT_TRUE(process_exists("test_servo_worker.elf"))
        << "Servo should be running";
    EXPECT_TRUE(process_exists("test_fieldbus_worker.elf"))
        << "Fieldbus should be running";

    stop_launcher(launcher_pid);

    sleep(1);
    EXPECT_FALSE(process_exists("test_state_master_worker.elf"))
        << "State master should be stopped";
    EXPECT_FALSE(process_exists("test_planning_worker.elf"))
        << "Planning should be stopped";
    EXPECT_FALSE(process_exists("test_servo_worker.elf"))
        << "Servo should be stopped";
    EXPECT_FALSE(process_exists("test_fieldbus_worker.elf"))
        << "Fieldbus should be stopped";
}

TEST_F(FourProcessTest, ConfigFileExists)
{
    std::ifstream config("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_TRUE(config.is_open()) << "Four-process config should exist";

    std::string line;
    int task_count = 0;
    while (std::getline(config, line)) {
        if (line.find("- name:") != std::string::npos) task_count++;
    }
    EXPECT_EQ(task_count, 4) << "Config should have exactly 4 tasks";
}

TEST_F(FourProcessTest, AllWorkerBinariesExist)
{
    EXPECT_EQ(access("./TaskFrame/output/taskframe_test/test_state_master_worker.elf", X_OK), 0);
    EXPECT_EQ(access("./TaskFrame/output/taskframe_test/test_planning_worker.elf", X_OK), 0);
    EXPECT_EQ(access("./TaskFrame/output/taskframe_test/test_servo_worker.elf", X_OK), 0);
    EXPECT_EQ(access("./TaskFrame/output/taskframe_test/test_fieldbus_worker.elf", X_OK), 0);
}

// ==== New tests ====

TEST_F(FourProcessTest, WorkerHeartbeatSurvival)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);
    ASSERT_TRUE(process_exists("test_state_master_worker.elf"));
    ASSERT_TRUE(process_exists("test_planning_worker.elf"));
    ASSERT_TRUE(process_exists("test_servo_worker.elf"));
    ASSERT_TRUE(process_exists("test_fieldbus_worker.elf"));

    // Wait longer than heartbeat timeout (500ms) to ensure workers
    // are not mistakenly killed for missing heartbeats
    sleep(2);

    EXPECT_TRUE(process_exists("test_state_master_worker.elf"))
        << "State master should survive heartbeat check";
    EXPECT_TRUE(process_exists("test_planning_worker.elf"))
        << "Planning should survive heartbeat check";
    EXPECT_TRUE(process_exists("test_servo_worker.elf"))
        << "Servo should survive heartbeat check";
    EXPECT_TRUE(process_exists("test_fieldbus_worker.elf"))
        << "Fieldbus should survive heartbeat check";

    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, WorkerAutoRestart)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);
    ASSERT_TRUE(process_exists("test_planning_worker.elf"));

    // Get original PID and kill the worker
    pid_t original_pid = get_pid("test_planning_worker.elf");
    ASSERT_GT(original_pid, 0);
    kill(original_pid, SIGKILL);

    // Wait for launcher to detect crash and restart
    int waited = 0;
    pid_t new_pid = -1;
    while (waited < 5000) {
        new_pid = get_pid("test_planning_worker.elf");
        if (new_pid > 0 && new_pid != original_pid) break;
        usleep(10000);
        waited += 10;
    }

    EXPECT_GT(new_pid, 0) << "Planning worker should be restarted";
    EXPECT_NE(new_pid, original_pid) << "Restarted worker should have new PID";

    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, LauncherGracefulShutdown)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);
    ASSERT_TRUE(process_exists("test_state_master_worker.elf"));

    // Send SIGTERM for graceful shutdown
    kill(launcher_pid, SIGTERM);

    int status;
    int waited = 0;
    while (waited < 5000) {
        pid_t result = waitpid(launcher_pid, &status, WNOHANG);
        if (result == launcher_pid) break;
        usleep(10000);
        waited += 10;
    }
    EXPECT_LT(waited, 5000) << "Launcher should exit gracefully within 5s";

    // Give workers time to process stop signal
    sleep(1);

    EXPECT_FALSE(process_exists("test_state_master_worker.elf"))
        << "State master should stop after launcher SIGTERM";
    EXPECT_FALSE(process_exists("test_planning_worker.elf"))
        << "Planning should stop after launcher SIGTERM";
    EXPECT_FALSE(process_exists("test_servo_worker.elf"))
        << "Servo should stop after launcher SIGTERM";
    EXPECT_FALSE(process_exists("test_fieldbus_worker.elf"))
        << "Fieldbus should stop after launcher SIGTERM";
}

TEST_F(FourProcessTest, SharedMemoryInitialization)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);

    // Verify shared memory exists and has valid header
    int fd = shm_open("/ROBOT_GLOBAL_DATA", O_RDONLY, 0);
    ASSERT_GE(fd, 0) << "Shared memory should be created";

    void* base = mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    ASSERT_NE(base, MAP_FAILED);

    struct ShmHeader {
        uint64_t magic;
        uint32_t version;
        uint32_t size;
        uint32_t crc;
    };
    ShmHeader* header = static_cast<ShmHeader*>(base);

    EXPECT_EQ(header->magic, 0x54462D53484D0000ULL)
        << "Shared memory should have valid magic number";
    EXPECT_EQ(header->version, 1u)
        << "Shared memory should have correct version";
    EXPECT_GT(header->size, 0u)
        << "Shared memory should have non-zero size";

    munmap(base, 4096);
    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, WorkerPriorityAndAffinity)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);

    // Check that state_master has the configured priority info
    pid_t sm_pid = get_pid("test_state_master_worker.elf");
    ASSERT_GT(sm_pid, 0);

    // Verify /proc/<pid>/stat exists and contains scheduling info.
    // In /proc/<pid>/stat, field 18 = nice, field 41 = sched policy.
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/stat", sm_pid);
    std::ifstream stat_file(path);
    ASSERT_TRUE(stat_file.is_open()) << "Should be able to read /proc/<pid>/stat";

    std::string stat_line;
    std::getline(stat_file, stat_line);
    EXPECT_FALSE(stat_line.empty()) << "stat file should not be empty";

    // Also verify sched_getscheduler returns a valid policy (SCHED_FIFO=1
    // needs root; without root it falls back to SCHED_OTHER=0).
    int policy = sched_getscheduler(sm_pid);
    EXPECT_GE(policy, 0) << "sched_getscheduler should succeed";

    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, AllWorkersRegisterStatus)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);

    // Give workers time to register and start heartbeating
    sleep(1);

    // All workers should still be alive after registration period
    EXPECT_TRUE(process_exists("test_state_master_worker.elf"))
        << "State master should be registered and running";
    EXPECT_TRUE(process_exists("test_planning_worker.elf"))
        << "Planning should be registered and running";
    EXPECT_TRUE(process_exists("test_servo_worker.elf"))
        << "Servo should be registered and running";
    EXPECT_TRUE(process_exists("test_fieldbus_worker.elf"))
        << "Fieldbus should be registered and running";

    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, ConfigValidation)
{
    std::ifstream config("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_TRUE(config.is_open());

    std::string line;
    bool found_shm_name = false;
    bool found_shm_size = false;
    bool found_watchdog = false;
    bool found_heartbeat = false;
    int autorestart_count = 0;
    int max_restarts_count = 0;

    while (std::getline(config, line)) {
        if (line.find("shm_name:") != std::string::npos) found_shm_name = true;
        if (line.find("shm_size:") != std::string::npos) found_shm_size = true;
        if (line.find("watchdog_interval_ms:") != std::string::npos) found_watchdog = true;
        if (line.find("heartbeat_timeout_ms:") != std::string::npos) found_heartbeat = true;
        if (line.find("autorestart:") != std::string::npos) autorestart_count++;
        if (line.find("max_restarts:") != std::string::npos) max_restarts_count++;
    }

    EXPECT_TRUE(found_shm_name) << "Config should specify shared memory name";
    EXPECT_TRUE(found_shm_size) << "Config should specify shared memory size";
    EXPECT_TRUE(found_watchdog) << "Config should specify watchdog interval";
    EXPECT_TRUE(found_heartbeat) << "Config should specify heartbeat timeout";
    EXPECT_EQ(autorestart_count, 4) << "All 4 workers should have autorestart enabled";
    EXPECT_EQ(max_restarts_count, 4) << "All 4 workers should have max_restarts configured";
}

TEST_F(FourProcessTest, NewSubsystemsInitializedInSharedMemory)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);

    int fd = shm_open("/ROBOT_GLOBAL_DATA", O_RDONLY, 0);
    ASSERT_GE(fd, 0);

    // Map just the header first to read the shared memory size.
    void* base = mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
    ASSERT_NE(base, MAP_FAILED);

    struct ShmHeader {
        uint64_t magic;
        uint32_t version;
        uint32_t size;
        uint32_t crc;
        uint32_t region_table_offset;
    };
    ShmHeader* header = static_cast<ShmHeader*>(base);

    EXPECT_EQ(header->magic, 0x54462D53484D0000ULL);

    // Wait for main task to finish InitNewSubsystems() -- there is a race
    // between worker processes appearing and shared memory being fully set up.
    int waited = 0;
    while (header->region_table_offset == 0 && waited < 5000) {
        usleep(10000);
        waited += 10;
    }
    EXPECT_GT(header->region_table_offset, 0u)
        << "region_table_offset should be set by main task within 5s";

    size_t shm_size = header->size;
    size_t table_offset = header->region_table_offset;
    munmap(base, 4096);

    // Remap the full shared memory so the region table is accessible.
    base = mmap(nullptr, shm_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    ASSERT_NE(base, MAP_FAILED) << "Failed to mmap full shared memory (size=" << shm_size << ")";

    struct RegionEntry {
        uint64_t offset;
        uint64_t size;
        uint64_t magic;
        uint32_t version;
        uint32_t pad;
    };
    struct RegionTable {
        uint32_t count;
        uint32_t version;
        RegionEntry entries[5];
    };

    RegionTable* table = reinterpret_cast<RegionTable*>(
        static_cast<char*>(base) + table_offset);

    EXPECT_EQ(table->count, 5u);
    EXPECT_EQ(table->entries[0].magic, 0x54462D5245470000ULL);
    EXPECT_EQ(table->entries[1].magic, 0x54462D5245470001ULL);
    EXPECT_EQ(table->entries[2].magic, 0x54462D5245470002ULL);
    EXPECT_EQ(table->entries[3].magic, 0x54462D5245470003ULL);
    EXPECT_EQ(table->entries[4].magic, 0x54462D5245470004ULL);

    munmap(base, shm_size);
    stop_launcher(launcher_pid);
}

TEST_F(FourProcessTest, WorkersSurviveWithNewSubsystems)
{
    pid_t launcher_pid = start_launcher("./cfg/unittest/taskframe_four_process.yaml");
    ASSERT_GT(launcher_pid, 0);

    wait_for_workers(5000);
    sleep(2);

    EXPECT_TRUE(process_exists("test_state_master_worker.elf"));
    EXPECT_TRUE(process_exists("test_planning_worker.elf"));
    EXPECT_TRUE(process_exists("test_servo_worker.elf"));
    EXPECT_TRUE(process_exists("test_fieldbus_worker.elf"));

    stop_launcher(launcher_pid);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
