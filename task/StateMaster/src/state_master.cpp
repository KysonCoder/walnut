#include "state_master.hpp"
#include "logger.h"

FSM_INITIAL_STATE(BaseFSM<CStateMasterTask>, UNINITIATED_STATE<CStateMasterTask>);

namespace {
// 同步窗口（UpdateTaskStatus 周期数，64ms/周期）
constexpr int kDefaultSyncWaitLimit = 80;   // ≈5.1s
constexpr int kInitSyncWaitLimit = 300;     // ≈19.2s：INIT 含 EtherCAT 冷启动
}  // namespace

CStateMasterTask::CStateMasterTask()
    : _expected_status(UNINTIATED),
      _other_task_syn_wait(0),
      _sync_wait_limit(kDefaultSyncWaitLimit),
      _subtask_error_mask(0),
      _manual_actionlist(nullptr),
      _program_actionlist(nullptr)
{
}

CStateMasterTask::~CStateMasterTask()
{
}

bool CStateMasterTask::InitTask(char* task_name)
{
    if (!CTaskFrame::InitTask(task_name)) {
        Log_E("CStateMasterTask::InitTask: CTaskFrame::InitTask failed\n");
        return false;
    }

    SetActionList(CTaskFrame::GetManualActionList(), CTaskFrame::GetProgramActionList());

    _fsm::setTask(this);
    _fsm::start();

    return true;
}

TASK_STATUS_TYPE CStateMasterTask::GetFSMState()
{
    if (_fsm::is_in_state<UNINITIATED_STATE<CStateMasterTask>>()) {
        return UNINTIATED;
    }
    if (_fsm::is_in_state<UNCONFIGED_STATE<CStateMasterTask>>()) {
        return UNCONFIGED;
    }
    if (_fsm::is_in_state<MOTOROFF_STATE<CStateMasterTask>>()) {
        return MOTOR_OFF;
    }
    if (_fsm::is_in_state<READY_STATE<CStateMasterTask>>()) {
        return READY;
    }
    if (_fsm::is_in_state<MOTION_STATE<CStateMasterTask>>()) {
        return RUNNING;
    }
    if (_fsm::is_in_state<ERROR_STATE<CStateMasterTask>>()) {
        return ERROR;
    }
    if (_fsm::is_in_state<ESTOP_STATE<CStateMasterTask>>()) {
        return EMERGENCY_STOP;
    }
    return UNINTIATED;
}

void CStateMasterTask::DispatchEvent(EVENT_TYPE event)
{
    if (event == EMPTY_EVENT) {
        return;
    }
    // 组合位掩码按优先级从高到低依次派发；当前 FSM 状态不 react 的事件被忽略
    if (event & EMERGENCY_STOP_EVENT) _fsm::dispatch(FSM_Emergency_Stop_Event());
    if (event & ERROR_EVENT)          _fsm::dispatch(FSM_Error_Event());
    if (event & RESET_EVENT)          _fsm::dispatch(FSM_Reset_Event());
    if (event & CONTROL_OFF_EVENT)    _fsm::dispatch(FSM_ControlOff_Event());
    if (event & CONTROL_ON_EVENT)     _fsm::dispatch(FSM_ControlOn_Event());
    if (event & MOTOR_OFF_EVENT)      _fsm::dispatch(FSM_MotorOff_Event());
    if (event & MOTOR_ON_EVENT)       _fsm::dispatch(FSM_MotorOn_Event());
    if (event & CONFIG_EVENT)         _fsm::dispatch(FSM_Config_Event());
    if (event & INIT_EVENT)           _fsm::dispatch(FSM_Init_Event());

    const int known_mask = EMERGENCY_STOP_EVENT | ERROR_EVENT | RESET_EVENT
                         | CONTROL_OFF_EVENT | CONTROL_ON_EVENT | MOTOR_OFF_EVENT
                         | MOTOR_ON_EVENT | CONFIG_EVENT | INIT_EVENT;
    if (event & ~known_mask) {
        Log_W("CStateMasterTask::DispatchEvent: unhandled event bits %d\n",
              event & ~known_mask);
    }
}

void CStateMasterTask::ScanSubtaskErrors()
{
    // 主任务已在 ERROR/ESTOP 时不扫描（2026-09-10 实测回归修复）：
    // ① 进入 ERROR 时 ErrorAction 已预置全掩码，恢复窗口内子任务合法滞留 ERROR；
    // ② 若此时清除掩码位（子任务尚未写上 ERROR 的窗口期），恢复窗口会把滞留
    //    ERROR 误判为新错误 → 重复上抛 → 全系统锁死在 ERROR（multi_process
    //    Reset_FromError 实测定律）。ESTOP 为锁存终态，同理不扫描。
    if (_fsm::is_in_state<ERROR_STATE<CStateMasterTask>>() ||
        _fsm::is_in_state<ESTOP_STATE<CStateMasterTask>>()) {
        return;
    }

    CStatusManager& status_mgr = GetStatusManager();
    int task_number = status_mgr.GetTaskNumber();

    for (int i = 0; i < task_number; i++) {
        // Skip StateMaster itself — only check sub-tasks
        if (i == GetTaskId()) {
            continue;
        }
        int status_value = 0;
        if (status_mgr.GetStatus(i, status_value)) {
            const uint32_t seen_bit = 1u << i;
            if (static_cast<TASK_STATUS_TYPE>(status_value) == ERROR) {
                // 边沿检测：同一轮 ERROR 只处置一次；RESET 恢复期间子任务
                // 合法地停留在 ERROR 直到消费 RESET，不得重复触发。
                if (!(_subtask_error_mask & seen_bit)) {
                    _subtask_error_mask |= seen_bit;
                    Log_E("CStateMasterTask::UpdateTaskStatus: sub-task %d in ERROR, entering ERROR state\n", i);
                    _other_task_syn_wait = 0;
                    GetEventManager().PublishBroadcast(EventType::ERROR);
                    _fsm::dispatch(FSM_Error_Event());
                    return;
                }
            } else {
                _subtask_error_mask &= ~seen_bit;  // 已恢复，允许下次再检测
            }
        }
    }
}

void CStateMasterTask::UpdateTaskStatus()
{
    // 子任务 ERROR 常开扫描（2026-09-10）：稳态下也必须上抛，
    // 不再被同步等待窗口挡住（原 `_other_task_syn_wait <= 0` 前置守卫的缺陷）
    ScanSubtaskErrors();

    if (_other_task_syn_wait <= 0) {
        return;
    }

    CStatusManager& status_mgr = GetStatusManager();
    int task_number = status_mgr.GetTaskNumber();
    int matched_count = 0;

    for (int i = 0; i < task_number; i++) {
        // Skip StateMaster itself — only check sub-tasks
        if (i == GetTaskId()) {
            continue;
        }
        int status_value = 0;
        if (status_mgr.GetStatus(i, status_value)) {
            if (static_cast<TASK_STATUS_TYPE>(status_value) == _expected_status) {
                matched_count++;
            }
        }
    }

    // matched_count must cover all OTHER tasks (excluding StateMaster)
    if (matched_count >= task_number - 1) {
        _other_task_syn_wait = 0;
        SetTaskStatus(_expected_status);
        // INIT 的加宽窗口在同步成功后还原，后续切换回到默认 watchdog 灵敏度
        if (_sync_wait_limit == kInitSyncWaitLimit) {
            _sync_wait_limit = kDefaultSyncWaitLimit;
        }
    } else {
        _other_task_syn_wait++;
        if (_other_task_syn_wait > _sync_wait_limit) {
            Log_E("CStateMasterTask::UpdateTaskStatus: timeout (limit %d) waiting for sub-tasks, broadcasting ERROR\n",
                  _sync_wait_limit);
            GetEventManager().PublishBroadcast(EventType::ERROR);
            _other_task_syn_wait = 0;
            if (_sync_wait_limit == kInitSyncWaitLimit) {
                _sync_wait_limit = kDefaultSyncWaitLimit;
            }
        }
    }
}

bool CStateMasterTask::SetActionList(util::RingBufferIpc<ActionGroupSharedData, 8>* manual_actionlist,
                                     util::RingBufferIpc<ActionGroupSharedData, 32>* program_actionlist)
{
    _manual_actionlist = manual_actionlist;
    _program_actionlist = program_actionlist;
    return true;
}

void CStateMasterTask::AddManualAction(const ActionGroupSharedData& action_data)
{
    if (_manual_actionlist != nullptr) {
        _manual_actionlist->TryPush(action_data);
    }
}

void CStateMasterTask::AddProgramAction(const ActionGroupSharedData& action_data)
{
    if (_program_actionlist != nullptr) {
        _program_actionlist->TryPush(action_data);
    }
}

int CStateMasterTask::InitAction()
{
    Log_D("CStateMasterTask::InitAction\n");
    // INIT 含 Fieldbus 的 EtherCAT 冷启动（主站激活 + 等 OP + SDO 读伺服），
    // 实测超过默认 5.1s 窗口。仅在窗口仍为默认值时放宽；
    // 显式 SetSyncWaitLimit() 的配置（如测试）优先。
    if (_sync_wait_limit == kDefaultSyncWaitLimit) {
        _sync_wait_limit = kInitSyncWaitLimit;
    }
    _expected_status = UNCONFIGED;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::INIT);
    return 0;
}

int CStateMasterTask::ConfigAction()
{
    Log_D("CStateMasterTask::ConfigAction\n");
    _expected_status = MOTOR_OFF;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::CONFIG);
    return 0;
}

int CStateMasterTask::MotorOnAction()
{
    Log_D("CStateMasterTask::MotorOnAction\n");
    _expected_status = READY;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::MOTOR_ON);
    return 0;
}

int CStateMasterTask::MotorOffAction()
{
    Log_D("CStateMasterTask::MotorOffAction\n");
    _expected_status = MOTOR_OFF;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::MOTOR_OFF);
    return 0;
}

int CStateMasterTask::ControlOnAction()
{
    Log_D("CStateMasterTask::ControlOnAction\n");
    _expected_status = RUNNING;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::CONTROL_ON);
    return 0;
}

int CStateMasterTask::ControlOffAction()
{
    Log_D("CStateMasterTask::ControlOffAction\n");
    _expected_status = READY;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::CONTROL_OFF);
    return 0;
}

int CStateMasterTask::ErrorAction()
{
    Log_D("CStateMasterTask::ErrorAction\n");
    _expected_status = ERROR;
    _other_task_syn_wait = 0;
    // 主任务主动进入 ERROR：所有子任务 ERROR 视为已知（它们本应进入 ERROR）。
    // 预置位掩码，避免 RESET 恢复期间把子任务合法的 ERROR 态误判为新错误。
    // 子任务恢复到非 ERROR 状态时其位自动清除，检测重新武装。
    _subtask_error_mask = 0xFFFFFFFFu;
    SetTaskStatus(_expected_status);
    GetEventManager().PublishBroadcast(EventType::ERROR);
    return 0;
}

int CStateMasterTask::ResetAction()
{
    Log_D("CStateMasterTask::ResetAction\n");
    _expected_status = MOTOR_OFF;
    _other_task_syn_wait = 1;
    GetEventManager().PublishBroadcast(EventType::RESET);
    return 0;
}

int CStateMasterTask::EstopAction()
{
    Log_D("CStateMasterTask::EstopAction\n");
    _expected_status = EMERGENCY_STOP;
    // ESTOP 是锁存终态，不等待子任务到位（对齐 ErrorAction）：
    // fieldbus 规划停期间合法停留 RUNNING，停完自行上报 EMERGENCY_STOP；
    // 等待只会把正常的减速过程误判为超时 ERROR。
    _other_task_syn_wait = 0;
    // 主任务自身立即上报急停（ErrorAction 同款模式）
    SetTaskStatus(EMERGENCY_STOP);
    GetEventManager().PublishBroadcast(EventType::EMERGENCY_STOP);
    return 0;
}
