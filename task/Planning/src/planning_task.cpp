/**
 * @file planning_task.cpp
 * @brief Planning task implementation
 */

#include "planning_task.hpp"
#include "main_fsm.hpp"
#include "logger.h"

#ifdef PLANNING_TASK_TEST
extern PLANNING_TASK_TEST_DATA _planning_task_test_data;
#endif

FSM_INITIAL_STATE(BaseFSM<CPlanningTask>, UNINITIATED_STATE<CPlanningTask>);

CPlanningTask::CPlanningTask()
    : _running_mode(MANUAL_MODE),
      _is_first_motion(true) {
}

CPlanningTask::~CPlanningTask() {
}

bool CPlanningTask::InitTask(char* task_name) {
    bool result = CTaskFrame::InitTask(task_name);
    if (!result) {
        Log_E("CPlanningTask::InitTask: CTaskFrame::InitTask failed\n");
        return false;
    }

    _fsm::setTask(this);
    _fsm::start();

    // RobotConfigStore 是单例；基类 InitTask 已完成 attach。
    if (!_runner.Init(robot_config::RobotConfigStore::Instance(), GetTrajectoryList())) {
        Log_E("CPlanningTask::InitTask: PlanningRunner init failed\n");
        return false;
    }

    _running_mode = MANUAL_MODE;
    _is_first_motion = true;

    return true;
}

void CPlanningTask::DispatchEvent(EVENT_TYPE event) {
    FSM_Init_Event init_event;
    FSM_Config_Event config_event;
    FSM_MotorOn_Event motor_on_event;
    FSM_MotorOff_Event motor_off_event;
    FSM_ControlOn_Event control_on_event;
    FSM_ControlOff_Event control_off_event;
    FSM_Error_Event error_event;
    FSM_Reset_Event reset_event;
    FSM_Emergency_Stop_Event emergency_stop_event;

    if (event & EVENT_TYPE::EMERGENCY_STOP_EVENT) {
        _fsm::dispatch(emergency_stop_event);
    }
    if (event & EVENT_TYPE::INIT_EVENT) {
        _fsm::dispatch(init_event);
    }
    if (event & EVENT_TYPE::CONFIG_EVENT) {
        _fsm::dispatch(config_event);
    }
    if (event & EVENT_TYPE::MOTOR_ON_EVENT) {
        _fsm::dispatch(motor_on_event);
    }
    if (event & EVENT_TYPE::MOTOR_OFF_EVENT) {
        _fsm::dispatch(motor_off_event);
    }
    if (event & EVENT_TYPE::CONTROL_ON_EVENT) {
        _fsm::dispatch(control_on_event);
    }
    if (event & EVENT_TYPE::CONTROL_OFF_EVENT) {
        _fsm::dispatch(control_off_event);
    }
    if (event & EVENT_TYPE::ERROR_EVENT) {
        _fsm::dispatch(error_event);
    }
    if (event & EVENT_TYPE::RESET_EVENT) {
        _fsm::dispatch(reset_event);
    }
}

void CPlanningTask::UpdateTaskStatus() {
    if (_fsm::is_in_state<UNINITIATED_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::UNINTIATED);
    else if (_fsm::is_in_state<UNCONFIGED_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::UNCONFIGED);
    else if (_fsm::is_in_state<MOTOROFF_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::MOTOR_OFF);
    else if (_fsm::is_in_state<ERROR_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::ERROR);
    else if (_fsm::is_in_state<MOTION_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::RUNNING);
    else if (_fsm::is_in_state<READY_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::READY);
    else if (_fsm::is_in_state<ESTOP_STATE<CPlanningTask>>())
        SetTaskStatus(TASK_STATUS_TYPE::EMERGENCY_STOP);
    else
        Log_E("CPlanningTask::UpdateTaskStatus: Unknown state\n");
}

int CPlanningTask::InitAction() {
    Log_D("CPlanningTask::InitAction\n");
    return 0;
}

int CPlanningTask::ConfigAction() {
    Log_D("CPlanningTask::ConfigAction\n");
    return 0;
}

int CPlanningTask::MotorOnAction() {
    Log_D("CPlanningTask::MotorOnAction\n");
    return 0;
}

int CPlanningTask::MotorOffAction() {
    Log_D("CPlanningTask::MotorOffAction\n");
    _runner.AbortPending();  // 丢弃待发射残余与记账（环由 Servo Reset 排空，对账自愈）
    return 0;
}

int CPlanningTask::ControlOnAction() {
    Log_D("CPlanningTask::ControlOnAction\n");
    return 0;
}

int CPlanningTask::ControlOffAction() {
    Log_D("CPlanningTask::ControlOffAction\n");
    if (_running_mode == AUTO_MODE) {
        Log_D("CPlanningTask::ControlOffAction AUTO stop (stubbed)\n");
    } else if (_running_mode == MANUAL_MODE) {
        Log_D("CPlanningTask::ControlOffAction MANUAL stop (stubbed)\n");
    } else {
        Log_D("CPlanningTask::ControlOffAction REMOTE stop (stubbed)\n");
    }
    return 0;
}

int CPlanningTask::ErrorAction() {
    Log_D("CPlanningTask::ErrorAction\n");
    _runner.AbortPending();
    return 0;
}

int CPlanningTask::ResetAction() {
    Log_D("CPlanningTask::ResetAction\n");
    return 0;
}

void CPlanningTask::ReadyStateRun() {
    // Log_D("CPlanningTask::ReadyStateRun\n");

    _is_first_motion = true;

    if (GetTaskStatus() != RUNNING && GetTaskStatus() != READY) {
        SetLastTaskStatus(GetTaskStatus());
        SetTaskStatus(READY);
    }
}

void CPlanningTask::MotionStateRun() {
    // Log_D("CPlanningTask::MotionStateRun\n");

    if (_is_first_motion) {
        _is_first_motion = false;
    }

    // Read servo feedback (real TaskFrame interaction, kept)
    FEEDBACK_ALL_JOINTS feedback_joints;
    GetFeedbackManager().GetJoints(feedback_joints);
    // Stub: original code processed feedback_joints for planning
    (void)feedback_joints;

    if (_running_mode == AUTO_MODE) {
        // Log_D("CPlanningTask::MotionStateRun AUTO mode (stubbed)\n");
    } else if (_running_mode == MANUAL_MODE) {
        // Log_D("CPlanningTask::MotionStateRun MANUAL mode (stubbed)\n");
    }
}

void CPlanningTask::ErrorStateRun() {
    // Log_D("CPlanningTask::ErrorStateRun\n");
}

int CPlanningTask::MainLoop() {
    EVENT_TYPE current_event;
    GetEvent(current_event);

    if (current_event != EMPTY_EVENT) {
        DispatchEvent(current_event);
    }

    // Synchronize local status with the FSM state every cycle so that
    // CTaskFrame can track whether the pending event has been processed.
    UpdateTaskStatus();

    _fsm::current_state_ptr->run();

    TfMsgQueueManager& mq = GetMsgQueueManager();
    Message msg;
    int task_id = GetTaskId();
    while (task_id >= 0 && mq.Receive(task_id, msg)) {
        // 模式切换（2026-09-08 手动模式设计）：StateMaster 已在 READY/RUNNING
        // 拦截，此处防御性复查本任务 FSM 状态。
        if (msg.type == MsgType::MANUAL_MODE || msg.type == MsgType::AUTO_MODE) {
            if (!_fsm::is_in_state<READY_STATE<CPlanningTask>>() &&
                !_fsm::is_in_state<MOTION_STATE<CPlanningTask>>()) {
                _running_mode = (msg.type == MsgType::MANUAL_MODE) ? MANUAL_MODE : AUTO_MODE;
                _runner.AbortPending();  // 待发射残余与链记录一并失效
                Log_I("CPlanningTask: running mode -> %s\n",
                      _running_mode == MANUAL_MODE ? "MANUAL" : "AUTO");
            } else {
                Log_W("CPlanningTask: mode switch rejected in READY/MOTION\n");
            }
        } else {
            Log_D("PlanningTask: received msg type %d (ignored)\n",
                  static_cast<int>(msg.type));
        }
    }

    // ② 水位门控 + 模式环选择（2026-09-08 手动模式设计）：
    // 仅 READY/MOTION 消费（预取窗口）；MANUAL 消费 manual 环，AUTO 消费程序环。
    const bool consumable = _fsm::is_in_state<READY_STATE<CPlanningTask>>() ||
                            _fsm::is_in_state<MOTION_STATE<CPlanningTask>>();

    // 排障日志（2026-09-09 部署无运动）：环内有积压却不消费时，~1s 打印一次门控状态
    static int s_diag_divider = 0;
    {
        auto* ml = GetManualActionList();
        auto* pl = GetProgramActionList();
        const uint32_t backlog = (ml != nullptr ? ml->Size() : 0) +
                                 (pl != nullptr ? pl->Size() : 0);
        if (backlog > 0 && ++s_diag_divider >= 16) {
            s_diag_divider = 0;
            Log_I("PlanningTask diag: mode=%s consumable=%d pending=%d water=%.3f "
                  "manual=%u program=%u\n",
                  _running_mode == MANUAL_MODE ? "MANUAL" : "AUTO",
                  consumable ? 1 : 0, _runner.HasPending() ? 1 : 0,
                  _runner.BufferedTimeS(),
                  ml != nullptr ? ml->Size() : 0, pl != nullptr ? pl->Size() : 0);
        }
    }

    if (consumable) {
        if (_runner.HasPending()) {
            if (_runner.EmitPending() == PlanningRunner::EmitResult::kError) {
                Log_E("CPlanningTask::MainLoop: emit failed\n");
                DispatchEvent(ERROR_EVENT);
            }
        } else if (_runner.BufferedTimeS() < PlanningRunner::kTrajPrefetchTimeS) {
            ActionGroupSharedData group;
            bool got = false;
            if (_running_mode == MANUAL_MODE) {
                auto* list = GetManualActionList();
                got = (list != nullptr && list->TryPop(group));
            } else {
                auto* list = GetProgramActionList();
                got = (list != nullptr && list->TryPop(group));
            }
            if (got) {
                FEEDBACK_ALL_JOINTS feedback;
                GetFeedbackManager().GetJoints(feedback);
                const PlanningRunner::GroupSource src =
                    (_running_mode == MANUAL_MODE) ? PlanningRunner::GroupSource::kManual
                                                   : PlanningRunner::GroupSource::kProgram;
                if (!_runner.PlanActionGroup(group, feedback, src) ||
                    _runner.EmitPending() == PlanningRunner::EmitResult::kError) {
                    Log_E("CPlanningTask::MainLoop: planning failed: %s (soc %d)\n",
                          _runner.last_result().message, _runner.last_result().soc_index);
                    DispatchEvent(ERROR_EVENT);
                }
            }
        }
    }

    usleep(64000);
    return 0;
}
