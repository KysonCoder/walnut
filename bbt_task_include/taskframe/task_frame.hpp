/**
 * @file taskframe.hpp
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


#ifndef __TASK_FRAME_HPP__
#define __TASK_FRAME_HPP__






#include "shm_ptr.hpp"
#include "osbase/os.h"
#include "utilbase/logger.h"

#include "datasender.h"


#include "tf_types.hpp"
#include "status_manager.hpp"

#include "tf_event.hpp"
#include "tf_msg_queue.hpp"
#include "tf_feedback.hpp"
#include "confbase/robot.hpp"
#include "confbase/joint_pos.hpp"

#include "utilbase/ring_buffer_ipc.hpp"
#include "utilbase/ring_buffer_ipc_data.hpp"
#include "actiongroup.hpp"
#include "microstep.hpp"
#include "trajectorygroup.hpp"
#include "engine_config.hpp"

// New subsystem forward declarations
class TfShmRegionManager;
class TfEventManager;
class TfMsgQueueManager;
class TfFeedbackManager;

#define SHARED_MEMORY_SIZE  1024*1024*100
#define SHARED_MEMORY_NAME  "/ROBOT_GLOBAL_DATA"
#define GLOBAL_DATA_MUTEX_NAME "/ROBOT_GLOBAL_DATA_MUTEX"




class CTaskDataBase
{
public:
    pthread_t   _thread_handle;         //thandle = return thread id
    int _stack_size = 8000000;            //stacksize   = expected task stack size
    char _task_name[32] = "default_task";    //taskname = created task name
    int _cpucore = 2;                   //cpucore = which cpu core the task will reside, value range: 1~16
    int _priority = 80;                 //priority = realtime task priority, value range: 1~99, 99 indicate the highest priority task

                                        // special data for each type of task
};


#ifdef TASK_FRAME_TEST

    typedef struct
    {
        /* data */
        EVENT_TYPE _event;
        TASK_STATUS_TYPE _status;
        CTaskDataBase _task_data;
    }TASK_FRAME_TEST_DATA;

#endif



typedef enum
{
    STOP_ON_PATH = 0,
    STOP_FAST = 1,
    STOP_EMERGENCY = 2,
    STOP_SAFE_POS = 3
}STOP_MODE_TYPE;



typedef struct
{
    int _shared_memory_id;
    void * _shared_data_ptr;
    char * _available_data_ptr;


    bool * _exit_flag;
    STOP_MODE_TYPE * _stop_mode;
    pthread_mutex_t * _global_mutex;
    int * _task_count;                          // shared task ID counter
    ShmPtr<StatusManagerData>  _task_status;
    ShmPtr<DataSenderData>     _data_sender;
    ShmPtr<util::RingBufferIpcData<ActionGroupSharedData, 32>> _program_list;
    ShmPtr<util::RingBufferIpcData<ActionGroupSharedData, 8>>  _manual_list;
    ShmPtr<util::RingBufferIpcData<PathSplineData, 64>> _trajectory_list;
    ShmPtr<util::RingBufferIpcData<ServoStepData, 16>>     _micro_list;
    ShmPtr<char>  _robot_config;       // RobotConfig region offset in shared memory
    size_t        _robot_config_size;  // RobotConfig region size (bytes)

}GlobalData;

enum class InitPhase : uint32_t {
    NONE = 0,
    SHM_OPENED = 1 << 0,
    GLOBAL_DATA_INITED = 1 << 1,
    ROBOT_CONFIG_LOADED = 1 << 2,
    SUBSYSTEMS_INITED = 1 << 3,
    TASK_REGISTERED = 1 << 4,
};

inline InitPhase operator|(InitPhase a, InitPhase b) {
    return static_cast<InitPhase>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline InitPhase operator&(InitPhase a, InitPhase b) {
    return static_cast<InitPhase>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
inline InitPhase& operator|=(InitPhase& a, InitPhase b) {
    a = a | b;
    return a;
}
inline bool HasPhase(InitPhase completed, InitPhase phase) {
    return (static_cast<uint32_t>(completed) & static_cast<uint32_t>(phase)) != 0;
}


class CTaskFrame
{
private:
    CTaskDataBase _task_data;   //param = parameter passed into the task

    bool _is_main_task; //is_main_task = true if the task is the main task, false otherwise
    // bool * _exit_flag; //exit_flag = true if the task should exit, false otherwise

    GlobalData * _task_shared_data;

    // Local copy of GlobalData for non-main tasks.  The absolute pointers
    // (_exit_flag, _stop_mode, _global_mutex) inside shared-memory GlobalData
    // are process-specific, so each non-main task keeps its own repaired
    // copy and points _task_shared_data here.  Main tasks use the shared
    // memory directly.
    GlobalData _local_shared_data;

    bool _new_subsystems_inited;
    bool CleanupAndFail(InitPhase completed);
    bool InitGlobalData();
    bool RegisterTask();
    bool InitNewSubsystems();
    bool AttachNewSubsystems();

    EVENT_TYPE _current_event;
    TASK_STATUS_TYPE _current_status;
    TASK_STATUS_TYPE _last_status;
    char _task_name[32];

    int _task_id;
    CStatusManager status_mgr_;
    CDataSender data_sender_;

    TfEventManager _tf_event_mgr;
    TfMsgQueueManager _tf_msg_queue_mgr;
    TfFeedbackManager _tf_feedback_mgr;
    robot_config::RobotConfigStore& _robot_config_store;

    // RingBufferIpc wrappers pointing into shared-memory data structs.
    util::RingBufferIpc<ActionGroupSharedData, 32>* program_action_list_;
    util::RingBufferIpc<ActionGroupSharedData, 8>*  manual_action_list_;
    util::RingBufferIpc<PathSplineData, 64>* trajectory_list_;
    util::RingBufferIpc<ServoStepData, 16>*    micro_list_;

    // Per-task event synchronization.  Each task consumes one event at a time
    // and waits up to kEventSyncTimeoutCycles for its FSM to reach the
    // expected target status before broadcasting an error.
    static constexpr int kEventSyncTimeoutCycles = 3;
    EVENT_TYPE _pending_event;
    TASK_STATUS_TYPE _pending_expected_status;
    int _event_sync_wait;
    bool _pending_event_dispatched;

public:
    CTaskFrame();
    ~CTaskFrame();

    void SetMainTask();
    bool IsMainTask() const { return _is_main_task; }
    bool OpenSharedMemory();

    util::RingBufferIpc<ActionGroupSharedData, 32> * GetProgramActionList();
    util::RingBufferIpc<ActionGroupSharedData, 8> * GetManualActionList();

    static GlobalData * GetGlobalData();
    char * GetTaskName();

    void SetExitFlag(bool flag);

    bool GetExitFlag();

    void GetTaskData(CTaskDataBase & taskData);

    void SetTaskData(CTaskDataBase & taskData);

    void GetEvent(EVENT_TYPE & event);

    virtual void SetTaskStatus(TASK_STATUS_TYPE task_status);
    virtual void SetLastTaskStatus(TASK_STATUS_TYPE task_status);
    virtual TASK_STATUS_TYPE GetTaskStatus();
    virtual TASK_STATUS_TYPE GetLastTaskStatus();

    STOP_MODE_TYPE GetStopMode();
    void SetStopMode(STOP_MODE_TYPE stop_mode);

    TfEventManager& GetEventManager();
    CStatusManager& GetStatusManager();
    TfMsgQueueManager& GetMsgQueueManager();
    TfFeedbackManager& GetFeedbackManager();
    CDataSender& GetDataSender();

    util::RingBufferIpc<ServoStepData, 16>* GetMicroList() { return micro_list_; }
    util::RingBufferIpc<PathSplineData, 64>* GetTrajectoryList() { return trajectory_list_; }

    // RobotConfig accessors
    bool BuildRobotConfig(const char* urdf_path, const char* cfg_path);
    bool AttachRobotConfig();
    void* GetRobotConfigBase() const;
    bool IsRobotConfigReady() const;

    const robot_config::UnitConfig* GetUnitConfig(int unit_type) const;
    const robot_config::RobotTopology* GetRobotTopology() const;
    const char* GetKinematicModelName(int unit_type) const;
    const char* GetRootLink(int unit_type) const;
    const char* GetTipLink(int unit_type) const;

    bool GetDynGroupNumber(int unit_type, int dyngrp_model,
                           int32_t* out_dyngrp_no) const;
    bool GetPowerRegainLimit(int unit_type, int dyngrp_model,
                             double* out_limit) const;

    CJointPos MakeJointPos(double* external_data) const;

    int GetTaskId() const { return _task_id; }

    virtual void UpdateTaskStatus();

    virtual bool InitTask(char * task_name);

    virtual int InitAction();
    virtual int ConfigAction();
    virtual int MotorOnAction();
    virtual int MotorOffAction();
    virtual int ControlOnAction();
    virtual int ControlOffAction();
    virtual int MainLoop();
    virtual int ErrorAction();
    virtual int ResetAction();
    virtual int EstopAction();       // 默认：ControlOffAction + MotorOffAction 链
    virtual int EstopResetAction();  // 默认：调用 ResetAction()

    virtual void ReadyStateRun();
    virtual void MotionStateRun();
    virtual void ErrorStateRun();
    virtual void EstopStateRun();

    // MOTION 收到 ERROR/ESTOP 时的规划停钩子。true=需要规划停（进入
    // MOTION_STOPPING_STATE）；false=无需停，直接进目标态。基类默认 false。
    virtual bool StartEmergencyPlannedStop();
    // MOTION_STOPPING_STATE 每周期执行；true=停完。基类默认 true。
    virtual bool EmergencyPlannedStopRun();

    // 暂停/继续/停止程序钩子（2026-10-10 停止暂停设计 §1）。
    // PauseAction: MOTION→PAUSED 前调用；0=允许暂停，非 0=拒绝（留 MOTION）。基类默认 0。
    virtual int PauseAction();
    // PausedStateRun: PAUSED 态每周期执行；基类默认上报 PAUSED 状态。
    virtual void PausedStateRun();
    // ResumeAction: PAUSED→MOTION 前调用；0=允许恢复，非 0=拒绝（留 PAUSED）。基类默认 0。
    virtual int ResumeAction();
    // PathStopAction: 停止程序（MOTION/PAUSED→READY；READY 中仅清环）。基类默认 0。
    virtual int PathStopAction();

    virtual void DispatchEvent(EVENT_TYPE event);

    // Returns true when motion has safely stopped.  Used when leaving
    // MOTION_STATE; default is always-stopped for stubs/tests.
    virtual bool IsMotionStopped() const;

    // Event → expected target status mapping for the per-task event sync
    // mechanism.  Public for testability.
    TASK_STATUS_TYPE ExpectedStatusForEvent(EVENT_TYPE event) const;
    bool EventRequiresLeavingMotion(EVENT_TYPE event) const;

};






#endif // __TASK_FRAME_HPP__
