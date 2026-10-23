#include <gtest/gtest.h>
#include "status_manager.hpp"

TEST(TaskStatusTest, TaskStatusTest1) {
    alignas(alignof(StatusManagerData))
    char buffer[sizeof(StatusManagerData)] = {};
    StatusManagerData* data = reinterpret_cast<StatusManagerData*>(buffer);

    CStatusManager::InitData(data);
    CStatusManager active_status(data);

    char task_name[32] = "task1";
    EXPECT_TRUE(active_status.NewTaskStatus(task_name));

    int event_value = 2;
    EXPECT_TRUE(active_status.SetStatus(task_name, event_value));

    int event_receive;
    EXPECT_TRUE(active_status.GetStatus(task_name, event_receive));
    EXPECT_EQ(event_receive, event_value);
}

TEST(TaskStatusTest, MultiProcessSimulation) {
    alignas(alignof(StatusManagerData))
    char buffer[sizeof(StatusManagerData)] = {};
    StatusManagerData* data = reinterpret_cast<StatusManagerData*>(buffer);

    CStatusManager::InitData(data);

    // Simulate process A
    CStatusManager mgr_a(data);
    mgr_a.NewTaskStatus("task_a");
    mgr_a.SetStatus("task_a", READY);

    // Simulate process B (only creates wrapper, no InitData)
    CStatusManager mgr_b(data);
    int status;
    EXPECT_TRUE(mgr_b.GetStatus("task_a", status));
    EXPECT_EQ(status, READY);
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
