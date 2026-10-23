#include <gtest/gtest.h>
#include "tf_fsm_adapter.hpp"

class MockTask {
public:
    int init_called = 0;
    int config_called = 0;
    int motor_on_called = 0;
    int InitAction() { init_called++; return 0; }
    int ConfigAction() { config_called++; return 0; }
    int MotorOnAction() { motor_on_called++; return 0; }
};

TEST(TfFsmAdapterTest, RegisterAndDispatch) {
    TfFsmEventAdapter adapter;
    MockTask task;
    adapter.RegisterEventHandler(EventType::INIT, [&](CTaskFrame*) { return task.InitAction(); }, TASK_STATUS_TYPE::UNCONFIGED);
    Event ev; ev.type = EventType::INIT;
    adapter.OnEventReceived(ev, nullptr);
    EXPECT_EQ(task.init_called, 1);
    EXPECT_EQ(adapter.GetExpectedState(EventType::INIT), TASK_STATUS_TYPE::UNCONFIGED);
}

TEST(TfFsmAdapterTest, UnregisteredEventIsNoOp) {
    TfFsmEventAdapter adapter;
    MockTask task;
    Event ev; ev.type = EventType::ERROR;
    adapter.OnEventReceived(ev, nullptr);
    EXPECT_EQ(task.init_called, 0);
    EXPECT_EQ(adapter.GetExpectedState(EventType::ERROR), TASK_STATUS_TYPE::UNDEFINED);
}

TEST(TfFsmAdapterTest, MultipleHandlers) {
    TfFsmEventAdapter adapter;
    MockTask task;
    adapter.RegisterEventHandler(EventType::INIT,    [&](CTaskFrame*) { return task.InitAction(); },    TASK_STATUS_TYPE::UNCONFIGED);
    adapter.RegisterEventHandler(EventType::CONFIG,  [&](CTaskFrame*) { return task.ConfigAction(); },  TASK_STATUS_TYPE::MOTOR_OFF);
    adapter.RegisterEventHandler(EventType::MOTOR_ON, [&](CTaskFrame*) { return task.MotorOnAction(); }, TASK_STATUS_TYPE::READY);
    Event ev1; ev1.type = EventType::INIT;
    Event ev2; ev2.type = EventType::CONFIG;
    Event ev3; ev3.type = EventType::MOTOR_ON;
    adapter.OnEventReceived(ev1, nullptr);
    adapter.OnEventReceived(ev2, nullptr);
    adapter.OnEventReceived(ev3, nullptr);
    EXPECT_EQ(task.init_called, 1);
    EXPECT_EQ(task.config_called, 1);
    EXPECT_EQ(task.motor_on_called, 1);
}

TEST(TfFsmAdapterTest, ClearAllHandlers) {
    TfFsmEventAdapter adapter;
    MockTask task;
    adapter.RegisterEventHandler(EventType::INIT, [&](CTaskFrame*) { return task.InitAction(); }, TASK_STATUS_TYPE::UNCONFIGED);
    adapter.ClearAllHandlers();
    Event ev; ev.type = EventType::INIT;
    adapter.OnEventReceived(ev, nullptr);
    EXPECT_EQ(task.init_called, 0);
    EXPECT_FALSE(adapter.HasHandler(EventType::INIT));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
