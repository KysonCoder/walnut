/**
 * @file taskframetest.cpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-12-31
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-12-31  0.1      Yonglin Chi       first edition
 */

// #define TASK_FRAME_TEST

#include <gtest/gtest.h>
#include "task_frame.hpp"

#include "tf_event.hpp"

#include <sys/types.h>

#include <sys/wait.h>



#include "task_frame.hpp"
#include "main_fsm.hpp"


extern ALARMLOG_LEVEL LOG_LEVEL;

CTaskFrame test_task;


#ifdef TASK_FRAME_TEST

extern TASK_FRAME_TEST_DATA task_frame_test_data;

#endif


// CRingBuffer<CActionGroup, 32> program_action_list;
// CRingBuffer<CActionGroup, 8> manual_action_list;

static void * TestTask(void * arg)
{
    Log_D("DummyTask start! \n");

    LOG_LEVEL = LOG_DEBUG;

    

    CTaskFrame * current_task = (CTaskFrame*)arg;

    CDataSender& _data_sender = current_task->GetDataSender();

    EVENT_TYPE _current_event;
    TASK_STATUS_TYPE _task_status;
    int _heart_beat = 0;
    
    while(!current_task->GetExitFlag())
    {
        current_task->GetEvent(_current_event);

        //Event_Dispatch(_current_event);

    #ifdef TASK_FRAME_TEST
        if(_current_event != EMPTY_EVENT)
            task_frame_test_data._event = _current_event;
    #endif

        _heart_beat++;

        // Log_I("DummyTask get event: %d \n",_heart_beat);

        _data_sender.writeData(1, (double) _heart_beat);

        _data_sender.writeError(1, "DummyTask", "error log test");

        if(_current_event != EMPTY_EVENT)
        {
            _task_status = (TASK_STATUS_TYPE)_current_event;
            current_task->SetTaskStatus(_task_status);
        }


        
    #ifdef TASK_FRAME_TEST
        task_frame_test_data._status = _task_status;
    #endif

        if(_heart_beat >= 200)
        {
            current_task->SetExitFlag(true);
        }
        
        os_sleep(16);
    }

    Log_D("DummyTask end! \n");

    
    return NULL;
}

bool TestTaskInit()
{
    char task_name[32] = "testtask";
    
    test_task.InitTask(task_name);
    
    CTaskDataBase task_data;
    test_task.GetTaskData(task_data);

    

    // #ifdef TASK_FRAME_TEST
    //     os_task_create(&task_data._thread_handle, task_data._stack_size, 
    //         (void*)TestTask, (void*)&test_task, task_data._task_name, task_data._cpucore);
    // #else
    //     os_task_create_rt(&task_data._thread_handle, task_data._stack_size, 
    //         (void*)DummyTask, (void*)&new_task, task_data._task_name, task_data._priority, task_data._cpucore);
    // #endif
    return true;
}





bool DummyTaskInit();

#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;

TEST(CTASKFRAME, constructor_test)
{

    
    TfEventManager& event_manager = test_task.GetEventManager();

    Event ev;
    bool has_event = event_manager.ConsumeEvent(0, ev);

    EXPECT_TRUE(has_event);
    EXPECT_EQ(ev.type, EventType::MOTOR_OFF);

    CStatusManager& task_status = test_task.GetStatusManager();

    int current_status;
    task_status.GetStatus(0,current_status);

    EXPECT_EQ((TASK_STATUS_TYPE) current_status, MOTOR_OFF);

    sleep(1);

    // test_task.SetExitFlag(true);




}

#endif


TEST(CTASKFRAME, event_manager_publish_consume)
{
    // test_task is already initialized by TestTaskInit() in main()
    TfEventManager& evt_mgr = test_task.GetEventManager();

    // Publish an event to task 0
    EXPECT_TRUE(evt_mgr.PublishEvent(0, EventType::INIT));
    EXPECT_TRUE(evt_mgr.PublishEvent(0, EventType::CONFIG));

    // Consume events
    Event ev1, ev2;
    EXPECT_TRUE(evt_mgr.ConsumeEvent(0, ev1));
    EXPECT_EQ(ev1.type, EventType::INIT);
    EXPECT_TRUE(evt_mgr.ConsumeEvent(0, ev2));
    EXPECT_EQ(ev2.type, EventType::CONFIG);

    // Queue should be empty now
    Event ev3;
    EXPECT_FALSE(evt_mgr.ConsumeEvent(0, ev3));
}

TEST(CTASKFRAME, event_manager_broadcast)
{
    // SPSC 规则：只有写端（主任务 new_task）可广播
    extern CTaskFrame new_task;
    TfEventManager& evt_mgr = new_task.GetEventManager();
    ASSERT_TRUE(evt_mgr.IsWriter());

    // Drain any stale events first
    Event drain;
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        while (evt_mgr.ConsumeEvent(i, drain)) {}
    }

    // Broadcast to all tasks
    EXPECT_TRUE(evt_mgr.PublishBroadcast(EventType::MOTOR_ON));

    for (int i = 0; i < 4; i++) {
        EXPECT_TRUE(evt_mgr.HasEvent(i));
        Event out;
        EXPECT_TRUE(evt_mgr.ConsumeEvent(i, out));
        EXPECT_EQ(out.type, EventType::MOTOR_ON);
    }
}

TEST(CTASKFRAME, msg_queue_send_receive)
{
    TfMsgQueueManager& mq_mgr = test_task.GetMsgQueueManager();

    Message msg;
    msg.type = MsgType::AUTO_MODE;
    msg.priority = 3;
    strcpy(msg.payload, "test_payload");
    msg.payload_size = 13;

    EXPECT_TRUE(mq_mgr.Send(0, msg.type, msg.payload, msg.payload_size, msg.priority));

    Message out;
    EXPECT_TRUE(mq_mgr.Receive(0, out));
    EXPECT_EQ(out.type, MsgType::AUTO_MODE);
    EXPECT_EQ(out.priority, 3u);
    EXPECT_STREQ(out.payload, "test_payload");
}

TEST(CTASKFRAME, get_event_returns_fifo_first_published)
{
    TfEventManager& evt_mgr = test_task.GetEventManager();
    const int task_id = test_task.GetTaskId();

    // Drain any stale events first.
    Event drain;
    while (evt_mgr.ConsumeEvent(task_id, drain)) {}

    // Publish events in FIFO order.
    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::INIT));
    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::MOTOR_ON));
    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::ERROR));

    EVENT_TYPE event = EMPTY_EVENT;
    test_task.GetEvent(event);
    EXPECT_EQ(event, INIT_EVENT);
    test_task.SetTaskStatus(UNCONFIGED);  // simulate FSM reaching expected status

    test_task.GetEvent(event);
    EXPECT_EQ(event, MOTOR_ON_EVENT);
    test_task.SetTaskStatus(READY);

    test_task.GetEvent(event);
    EXPECT_EQ(event, ERROR_EVENT);
    test_task.SetTaskStatus(ERROR);

    // Queue should now be empty.
    Event leftover;
    EXPECT_FALSE(evt_mgr.ConsumeEvent(task_id, leftover));
}

TEST(CTASKFRAME, get_event_returns_fifo_not_priority)
{
    TfEventManager& evt_mgr = test_task.GetEventManager();
    const int task_id = test_task.GetTaskId();

    Event drain;
    while (evt_mgr.ConsumeEvent(task_id, drain)) {}

    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::EMERGENCY_STOP));  // first
    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::CONFIG));          // second
    EXPECT_TRUE(evt_mgr.PublishEvent(task_id, EventType::RESET));           // third

    EVENT_TYPE event = EMPTY_EVENT;
    test_task.GetEvent(event);
    EXPECT_EQ(event, EMERGENCY_STOP_EVENT);
    test_task.SetTaskStatus(EMERGENCY_STOP);  // 急停的期望状态为 EMERGENCY_STOP

    test_task.GetEvent(event);
    EXPECT_EQ(event, CONFIG_EVENT);
    test_task.SetTaskStatus(MOTOR_OFF);       // CONFIG 的期望状态为 MOTOR_OFF

    test_task.GetEvent(event);
    EXPECT_EQ(event, RESET_EVENT);
    test_task.SetTaskStatus(MOTOR_OFF);

    Event leftover;
    EXPECT_FALSE(evt_mgr.ConsumeEvent(task_id, leftover));
}

TEST(CTASKFRAME, get_event_returns_empty_when_queue_empty)
{
    TfEventManager& evt_mgr = test_task.GetEventManager();
    const int task_id = test_task.GetTaskId();

    Event drain;
    while (evt_mgr.ConsumeEvent(task_id, drain)) {}

    EVENT_TYPE event = INIT_EVENT;
    test_task.GetEvent(event);

    EXPECT_EQ(event, EMPTY_EVENT);
}

TEST(CTASKFRAME, feedback_manager_update_get)
{
    TfFeedbackManager& fb_mgr = test_task.GetFeedbackManager();

    FEEDBACK_ALL_JOINTS data_in;
    memset(&data_in, 0, sizeof(data_in));
    for (int i = 0; i < (int)robot_config::kRobotMaxJoints; i++) {
        data_in.joints[i].position_feedback = 0.5 * i;
        data_in.joints[i].speed_feedback = 0.05 * i;
    }

    fb_mgr.UpdateJoints(data_in);

    FEEDBACK_ALL_JOINTS data_out;
    EXPECT_TRUE(fb_mgr.GetJoints(data_out));
    for (int i = 0; i < (int)robot_config::kRobotMaxJoints; i++) {
        EXPECT_DOUBLE_EQ(data_out.joints[i].position_feedback, 0.5 * i);
        EXPECT_DOUBLE_EQ(data_out.joints[i].speed_feedback, 0.05 * i);
    }
}

TEST(CTASKFRAME, init_test)
{


    GlobalData * global_data = test_task.GetGlobalData();

    CActionGroup current_action_group;

    auto* program_list = test_task.GetProgramActionList();
    if (program_list != nullptr) {
        ActionGroupSharedData shared_data;
        program_list->TryPop(shared_data);
        current_action_group.GetSharedData() = shared_data;
    }

    double path_length1;
    current_action_group.GetMasterPathLength(&path_length1);

    ASSERT_DOUBLE_EQ(path_length1, 1.17);
    sleep(1);

    test_task.SetExitFlag(true);

}

TEST(CTASKFRAME, expected_status_for_estop_event)
{
    CTaskFrame task;
    EXPECT_EQ(task.ExpectedStatusForEvent(EMERGENCY_STOP_EVENT), EMERGENCY_STOP);
    EXPECT_TRUE(task.EventRequiresLeavingMotion(EMERGENCY_STOP_EVENT));
}

int dummy_main();

int main(int argc, char *argv[])
{
    dummy_main();
    TestTaskInit();
    
	testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();

    // return 0;
}