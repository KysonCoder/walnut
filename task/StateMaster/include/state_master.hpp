#ifndef __STATE_MASTER_HPP__
#define __STATE_MASTER_HPP__

#include <atomic>

#include "task_frame.hpp"
#include "robot.hpp"
#include "main_fsm.hpp"
#include "actiongroup.hpp"

#ifdef STATE_MASTER_TEST
typedef struct {
    EVENT_TYPE _event;
    TASK_STATUS_TYPE _status;
    CTaskDataBase _task_data;
} STATE_MASTER_TEST_DATA;
#endif

class CStateMasterTask : public CTaskFrame {
private:
    TASK_STATUS_TYPE _expected_status;
    int _other_task_syn_wait;
    int _sync_wait_limit;  // UpdateTaskStatus 同步等待上限（周期数），默认 80
    uint32_t _subtask_error_mask;  // 已处置的子任务 ERROR 边沿检测位掩码（bit i = task i）
    CActionGroup _new_action_group;
    util::RingBufferIpc<ActionGroupSharedData, 8>* _manual_actionlist;
    util::RingBufferIpc<ActionGroupSharedData, 32>* _program_actionlist;
    // 运行模式记录（HTTP 线程写/读，需原子）：true=MANUAL（默认，与 Planning 一致）
    std::atomic<bool> _running_mode_manual{true};
    using _fsm = BaseFSM<CStateMasterTask>;

public:
    CStateMasterTask();
    ~CStateMasterTask();

    TASK_STATUS_TYPE GetFSMState();

    void SetRunningModeManual(bool manual) { _running_mode_manual.store(manual); }
    bool IsManualMode() const { return _running_mode_manual.load(); }

private:
    // 子任务 ERROR 常开扫描（每周期，与同步窗口无关）。2026-09-10 稳态上抛设计。
    void ScanSubtaskErrors();

public:

    util::RingBufferIpc<ActionGroupSharedData, 8>* GetManualActionList() { return _manual_actionlist; }
    util::RingBufferIpc<ActionGroupSharedData, 32>* GetProgramActionList() { return _program_actionlist; }

    bool InitTask(char* task_name) override;
    void DispatchEvent(EVENT_TYPE event) override;
    void UpdateTaskStatus() override;

    void SetSyncWaitLimit(int limit) { _sync_wait_limit = limit; }
    int GetSyncWaitLimit() const { return _sync_wait_limit; }

    bool SetActionList(util::RingBufferIpc<ActionGroupSharedData, 8>* manual_actionlist,
                       util::RingBufferIpc<ActionGroupSharedData, 32>* program_actionlist);
    void AddManualAction(const ActionGroupSharedData& action_data);
    void AddProgramAction(const ActionGroupSharedData& action_data);

    int InitAction() override;
    int ConfigAction() override;
    int MotorOnAction() override;
    int MotorOffAction() override;
    int ControlOnAction() override;
    int ControlOffAction() override;
    int ErrorAction() override;
    int ResetAction() override;
    int EstopAction() override;
};

#endif
