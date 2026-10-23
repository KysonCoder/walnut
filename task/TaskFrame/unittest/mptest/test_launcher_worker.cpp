#include <gtest/gtest.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <fstream>

// Helper: check if process exists by name
static bool process_exists(const char* name) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "pgrep -x %s > /dev/null 2>&1", name);
    return system(cmd) == 0;
}

// Helper: kill process by name
static void kill_process(const char* name) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "pkill -9 -x %s 2>/dev/null", name);
    system(cmd);
}

class LauncherWorkerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Clean up any stale shared memory or processes
        shm_unlink("/ROBOT_GLOBAL_DATA");
        kill_process("test_worker.elf");
        kill_process("taskframe_launcher");
        usleep(200000); // 200ms
    }

    void TearDown() override {
        kill_process("test_worker.elf");
        kill_process("taskframe_launcher");
        shm_unlink("/ROBOT_GLOBAL_DATA");
    }
};

TEST_F(LauncherWorkerTest, LauncherStartsWorker)
{
    pid_t launcher_pid = fork();
    ASSERT_GE(launcher_pid, 0) << "fork failed";

    if (launcher_pid == 0) {
        // Child: run launcher
        execl("./app/taskframe_launcher",
              "taskframe_launcher",
              "./cfg/unittest/taskframe_launcher.yaml",
              nullptr);
        _exit(1); // exec failed
    }

    // Parent: wait for worker to be spawned
    sleep(2);

    // Check that test_worker process exists
    bool worker_running = process_exists("test_worker.elf");
    EXPECT_TRUE(worker_running) << "Worker process should be running";

    // Stop launcher (sends SIGTERM to itself which triggers cleanup)
    kill(launcher_pid, SIGTERM);

    int status;
    int waited = 0;
    while (waited < 3000) {
        pid_t result = waitpid(launcher_pid, &status, WNOHANG);
        if (result == launcher_pid) break;
        usleep(10000);
        waited += 10;
    }

    if (waited >= 3000) {
        kill(launcher_pid, SIGKILL);
        waitpid(launcher_pid, &status, 0);
    }

    // Verify worker is gone
    sleep(1);
    EXPECT_FALSE(process_exists("test_worker.elf")) << "Worker should be stopped";
}

TEST_F(LauncherWorkerTest, LauncherConfigParse)
{
    // Verify config file exists and has expected structure
    std::ifstream config("./cfg/unittest/taskframe_launcher.yaml");
    ASSERT_TRUE(config.is_open()) << "Config file should exist";

    std::string line;
    bool found_shm = false;
    bool found_tasks = false;
    while (std::getline(config, line)) {
        if (line.find("shm_name:") != std::string::npos) found_shm = true;
        if (line.find("tasks:") != std::string::npos) found_tasks = true;
    }

    EXPECT_TRUE(found_shm) << "Config should have shm_name";
    EXPECT_TRUE(found_tasks) << "Config should have tasks section";
}

TEST_F(LauncherWorkerTest, LauncherBinaryExists)
{
    EXPECT_EQ(access("./app/taskframe_launcher", X_OK), 0)
        << "Launcher binary should exist and be executable";
}

TEST_F(LauncherWorkerTest, WorkerBinaryExists)
{
    EXPECT_EQ(access("./TaskFrame/output/taskframe_test/test_worker.elf", X_OK), 0)
        << "Test worker binary should exist and be executable";
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
