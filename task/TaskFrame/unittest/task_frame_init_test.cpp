#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <cstdlib>

#include "task_frame.hpp"
#include "shm_header.hpp"

TEST(CTaskFrameInit, ConstructorInitializesPointersToNull) {
    CTaskFrame task;
    EXPECT_EQ(task.GetGlobalData()->_shared_data_ptr, nullptr);
    EXPECT_EQ(task.GetGlobalData()->_exit_flag, nullptr);
    EXPECT_EQ(task.GetGlobalData()->_stop_mode, nullptr);
    EXPECT_EQ(task.GetGlobalData()->_global_mutex, nullptr);
    EXPECT_EQ(task.GetGlobalData()->_task_count, nullptr);
}

TEST(CTaskFrameInit, GetExitFlagReturnsTrueWhenUninitialized) {
    CTaskFrame task;
    EXPECT_TRUE(task.GetExitFlag());
}

TEST(CTaskFrameInit, SetExitFlagNoopWhenUninitialized) {
    CTaskFrame task;
    task.SetExitFlag(true);  // Should not crash.
    EXPECT_TRUE(task.GetExitFlag());
}

TEST(CTaskFrameInit, GetStopModeReturnsSafeDefaultWhenUninitialized) {
    CTaskFrame task;
    EXPECT_EQ(task.GetStopMode(), STOP_FAST);
}

TEST(CTaskFrameInit, InitTaskFailsGracefullyWhenConfigMissing) {
    CTaskFrame task;
    task.SetMainTask();

    // Force ConfigPathResolver to fail by pointing to non-existent files.
    system("mkdir -p ./cfg");
    std::ofstream bad_cfg("./cfg/taskframe_paths.cfg");
    bad_cfg << "urdf=./cfg/nonexistent.urdf\n";
    bad_cfg << "robot_cfg=./cfg/nonexistent.cfg\n";
    bad_cfg.close();

    char task_name[] = "testtask";
    EXPECT_FALSE(task.InitTask(task_name));

    std::remove("./cfg/taskframe_paths.cfg");
    rmdir("./cfg");
}

TEST(CTaskFrameInit, CleanupUnlinksSharedMemoryOnFailure) {
    // Pre-create shared memory to ensure shm_unlink runs.
    shm_unlink("/ROBOT_GLOBAL_DATA");
    int fd = shm_open("/ROBOT_GLOBAL_DATA", O_CREAT | O_RDWR, 0666);
    ASSERT_GE(fd, 0);
    close(fd);

    // Force ConfigPathResolver to fail regardless of whether default cfg files exist.
    system("mkdir -p ./cfg");
    std::ofstream bad_cfg("./cfg/taskframe_paths.cfg");
    bad_cfg << "urdf=./cfg/nonexistent.urdf\n";
    bad_cfg << "robot_cfg=./cfg/nonexistent.cfg\n";
    bad_cfg.close();

    {
        CTaskFrame task;
        task.SetMainTask();
        char task_name[] = "testtask";
        EXPECT_FALSE(task.InitTask(task_name));
    }

    int check_fd = shm_open("/ROBOT_GLOBAL_DATA", O_RDWR, 0);
    EXPECT_LT(check_fd, 0) << "Shared memory should have been unlinked after InitTask failure";
    if (check_fd >= 0) close(check_fd);
    g_shm_base = nullptr;
    g_shm_size = 0;
    std::memset(CTaskFrame::GetGlobalData(), 0, sizeof(GlobalData));
    shm_unlink("/ROBOT_GLOBAL_DATA");
    std::remove("./cfg/taskframe_paths.cfg");
}

TEST(CTaskFrameInit, ChildTaskRejectsInvalidShmHeader) {
    // Create a shm object with wrong magic.
    shm_unlink("/ROBOT_GLOBAL_DATA");
    int fd = shm_open("/ROBOT_GLOBAL_DATA", O_CREAT | O_RDWR, 0666);
    ASSERT_GE(fd, 0);
    ASSERT_EQ(ftruncate(fd, SHARED_MEMORY_SIZE), 0);

    void* base = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    ASSERT_NE(base, MAP_FAILED);

    ShmHeader* header = static_cast<ShmHeader*>(base);
    header->magic = 0xDEADBEEF;  // Invalid.
    header->version = 99;

    {
        CTaskFrame task;
        char task_name[] = "childtask";
        EXPECT_FALSE(task.InitTask(task_name));
    }

    // OpenSharedMemory() already munmapped on invalid header.
    g_shm_base = nullptr;
    g_shm_size = 0;
    std::memset(CTaskFrame::GetGlobalData(), 0, sizeof(GlobalData));
    shm_unlink("/ROBOT_GLOBAL_DATA");
}

TEST(CTaskFrameInit, ShmHeaderDefaultsToCurrentLayoutVersion) {
    // 2026-08-21 func/action 的 ActionData ABI 变更（sizeof/字段偏移改变），
    // 跨进程共享区域必须重建；布局版本随之升为 2。
    ShmHeader header;
    EXPECT_EQ(header.version, kShmLayoutVersion);
    EXPECT_EQ(kShmLayoutVersion, 2u);
}

TEST(CTaskFrameInit, ChildTaskRejectsStaleLayoutVersion) {
    // 旧版（v1）布局的残留 shm 必须被拒绝，即使 magic 正确。
    shm_unlink("/ROBOT_GLOBAL_DATA");
    int fd = shm_open("/ROBOT_GLOBAL_DATA", O_CREAT | O_RDWR, 0666);
    ASSERT_GE(fd, 0);
    ASSERT_EQ(ftruncate(fd, SHARED_MEMORY_SIZE), 0);

    void* base = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    ASSERT_NE(base, MAP_FAILED);

    ShmHeader* header = static_cast<ShmHeader*>(base);
    header->magic = SHM_MAGIC;
    header->version = 1;  // 旧布局版本。

    {
        CTaskFrame task;
        char task_name[] = "childtask";
        EXPECT_FALSE(task.InitTask(task_name));
    }

    g_shm_base = nullptr;
    g_shm_size = 0;
    std::memset(CTaskFrame::GetGlobalData(), 0, sizeof(GlobalData));
    shm_unlink("/ROBOT_GLOBAL_DATA");
}
