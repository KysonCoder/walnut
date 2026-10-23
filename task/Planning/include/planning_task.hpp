/**
 * @file planning_task.hpp
 * @brief Planning task — trajectory planning and interpolation coordinator
 */

#ifndef __PLANNING_TASK_HPP__
#define __PLANNING_TASK_HPP__

#include "task_frame.hpp"
#include "main_fsm.hpp"
#include "planning_runner.hpp"

#ifdef PLANNING_TASK_TEST

typedef struct {
    EVENT_TYPE _event;
    TASK_STATUS_TYPE _status;
    CTaskDataBase _task_data;
} PLANNING_TASK_TEST_DATA;

#endif

typedef enum {
    MANUAL_MODE = 0,
    AUTO_MODE = 1,
    REMOTE_MODE = 2
} RUNNING_MODE_TYPE;

class CPlanningTask : public CTaskFrame {
private:
    using _fsm = BaseFSM<CPlanningTask>;

    RUNNING_MODE_TYPE _running_mode;
    bool _is_first_motion;
    PlanningRunner _runner;

public:
    CPlanningTask();
    ~CPlanningTask();

    bool InitTask(char* task_name) override;
    void DispatchEvent(EVENT_TYPE event) override;
    void UpdateTaskStatus() override;

    // FSM action handlers
    int InitAction() override;
    int ConfigAction() override;
    int MotorOnAction() override;
    int MotorOffAction() override;
    int ControlOnAction() override;
    int ControlOffAction() override;
    int ErrorAction() override;
    int ResetAction() override;

    // Periodic state runs
    void ReadyStateRun() override;
    void MotionStateRun() override;
    void ErrorStateRun() override;

    // Main loop — one iteration per call
    int MainLoop() override;
};

#endif // __PLANNING_TASK_HPP__
