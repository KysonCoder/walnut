/**
 * @file task_frame.cpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2025-01-17
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2025-01-17  0.1      Yonglin Chi       first edition
 */

#include <fcntl.h>           // For O_* constants
#include <sys/mman.h>        // For shm_open, mmap
#include <sys/stat.h>        // For mode constants
#include <unistd.h>          // For ftruncate
#include <cstring>           // For std::memset

#include "task_frame.hpp"
#include "main_fsm.hpp"
#include "shm_header.hpp"
#include "servo_ref.h"

#include "tf_shm_region.hpp"
#include "tf_event.hpp"
#include "tf_msg_queue.hpp"
#include "tf_feedback.hpp"

#include "confbase/robot.hpp"
#include "confbase/joint_pos.hpp"

#include "config_path_resolver.hpp"

extern ALARMLOG_LEVEL LOG_LEVEL;

CTaskFrame new_task;


#ifdef TASK_FRAME_TEST

extern TASK_FRAME_TEST_DATA task_frame_test_data;

#endif

GlobalData _global_data;

// extern REF_RING reference_bufffer;

static void * DummyTask(void * arg)
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
        // current_task->GetEvent(_current_event);

        //Event_Dispatch(_current_event);

    #ifdef TASK_FRAME_TEST
        if(_current_event != EMPTY_EVENT)
            task_frame_test_data._event = _current_event;
    #endif

        _heart_beat++;

        // Log_I("DummyTask get event: %d \n",_heart_beat);

        // _data_sender->writeData(1, (double) _heart_beat);

        // _data_sender->writeError(1, "DummyTask", "error log test");

        // if(_current_event != EMPTY_EVENT)
        // {
        //     _task_status = (TASK_STATUS_TYPE)_current_event;
        //     current_task->SetTaskStatus(_task_status);
        // }


        
    #ifdef TASK_FRAME_TEST
        task_frame_test_data._status = _task_status;
    #endif

        // if(_heart_beat >= 200)
        // {
        //     current_task->SetExitFlag(true);
        // }
        
        os_sleep(16);
    }

    Log_D("DummyTask end! \n");

    
    return NULL;
}

bool DummyTaskInit()
{
    char task_name[32] = "dummytask";
    new_task.SetMainTask();
    if (!new_task.InitTask(task_name)) {
        Log_E("DummyTaskInit: InitTask failed\n");
        return false;
    }

    CTaskDataBase task_data;
    new_task.GetTaskData(task_data);



#ifdef TASK_FRAME_TEST
    os_task_create(&task_data._thread_handle, task_data._stack_size, 
        (void*)DummyTask, (void*)&new_task, task_data._task_name, task_data._cpucore);
#else
    os_task_create_rt(&task_data._thread_handle, task_data._stack_size, 
        (void*)DummyTask, (void*)&new_task, task_data._task_name, task_data._priority, task_data._cpucore);
#endif
    return true;
}


CTaskFrame::CTaskFrame()
    : _robot_config_store(robot_config::RobotConfigStore::Instance()) {
    _is_main_task = false;
    _current_status = UNINTIATED;
    _last_status = UNINTIATED;
    _new_subsystems_inited = false;
    _task_id = -1;
    _task_shared_data = nullptr;
    program_action_list_ = nullptr;
    manual_action_list_ = nullptr;
    trajectory_list_ = nullptr;
    micro_list_ = nullptr;
    _pending_event = EMPTY_EVENT;
    _pending_expected_status = UNINTIATED;
    _event_sync_wait = 0;
    _pending_event_dispatched = false;
}

CTaskFrame::~CTaskFrame()
{
    // Shared memory objects are not destructed here because they are shared
    // across processes. The launcher is responsible for shm_unlink.
    delete program_action_list_;
    delete manual_action_list_;
    delete trajectory_list_;
    delete micro_list_;
}

void CTaskFrame::SetMainTask()
{  
    _is_main_task = true;
    
}

util::RingBufferIpc<ActionGroupSharedData, 32> * CTaskFrame::GetProgramActionList()
{
    // 判定用包装指针而非 _global_data._program_list：子任务路径下
    // _global_data 是未填充的本地成员，ShmPtr 只在 _task_shared_data
    // （_local_shared_data 副本）里有效。
    if (program_action_list_ != nullptr)
        return program_action_list_;
    else
    {
        Log_D("GetProgramActionList failed!\n");
        return nullptr;
    }
}
util::RingBufferIpc<ActionGroupSharedData, 8> * CTaskFrame::GetManualActionList()
{
    if (manual_action_list_ != nullptr)
        return manual_action_list_;
    else
    {
        Log_D("GetManualActionList failed!\n");
        return nullptr;
    }
}

TfEventManager& CTaskFrame::GetEventManager() {
    return _tf_event_mgr;
}

CStatusManager& CTaskFrame::GetStatusManager() {
    return status_mgr_;
}

TfMsgQueueManager& CTaskFrame::GetMsgQueueManager() {
    return _tf_msg_queue_mgr;
}

TfFeedbackManager& CTaskFrame::GetFeedbackManager() {
    return _tf_feedback_mgr;
}

GlobalData * CTaskFrame::GetGlobalData()
{
    return &_global_data;
}

char * CTaskFrame::GetTaskName()
{
    return _task_name;
}

// Align pointer up to alignment boundary for type T
template<typename T>
static char* align_up(char* ptr) {
    uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
    uintptr_t alignment = alignof(T);
    return reinterpret_cast<char*>((addr + alignment - 1) & ~(alignment - 1));
}

static char* align_page(char* ptr) {
    uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
    uintptr_t page_size = static_cast<uintptr_t>(robot_config::kShmPageSize);
    return reinterpret_cast<char*>(((addr + page_size - 1) / page_size) * page_size);
}

bool CTaskFrame::InitGlobalData()
{
    if (_global_data._shared_data_ptr == NULL) {
        return false;
    }

    // memset(_global_data._shared_data_ptr,0,SHARED_MEMORY_SIZE);
    // Place GlobalData copy right after ShmHeader, then all objects after it
    _global_data._available_data_ptr = (char *) _global_data._shared_data_ptr
                                       + sizeof(ShmHeader) + sizeof(GlobalData);
    _global_data._exit_flag = (bool*) _global_data._available_data_ptr;
    * _global_data._exit_flag = false;
    _global_data._available_data_ptr += sizeof(bool);

    _global_data._stop_mode = (STOP_MODE_TYPE*) _global_data._available_data_ptr;
    _global_data._available_data_ptr += sizeof(STOP_MODE_TYPE);
    * _global_data._stop_mode = STOP_FAST;
    _global_data._available_data_ptr += sizeof(STOP_MODE_TYPE);

    _global_data._available_data_ptr = align_up<pthread_mutex_t>(_global_data._available_data_ptr);
    _global_data._global_mutex = (pthread_mutex_t*) _global_data._available_data_ptr;
    _global_data._available_data_ptr += sizeof(pthread_mutex_t);

    _global_data._available_data_ptr = align_up<int>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._task_count = (int*)(static_cast<char*>(_global_data._shared_data_ptr) + off);
        *_global_data._task_count = 0;
        _global_data._available_data_ptr += sizeof(int);
    }

    _global_data._available_data_ptr = align_up<StatusManagerData>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._task_status = ShmPtr<StatusManagerData>(off);
        _global_data._available_data_ptr += sizeof(StatusManagerData);
    }
    CStatusManager::InitData(_global_data._task_status.get());

    _global_data._available_data_ptr = align_up<DataSenderData>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._data_sender = ShmPtr<DataSenderData>(off);
        _global_data._data_sender.placement_new();
        CDataSender::InitData(_global_data._data_sender.get());
        _global_data._available_data_ptr += sizeof(DataSenderData);
    }

    _global_data._available_data_ptr = align_up<util::RingBufferIpcData<ActionGroupSharedData, 8>>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._manual_list = ShmPtr<util::RingBufferIpcData<ActionGroupSharedData, 8>>(off);
        for (int i = 0; i < MAX_ENGINE_NUMBER; i++) {
            new (_global_data._available_data_ptr + i * sizeof(util::RingBufferIpcData<ActionGroupSharedData, 8>))
                util::RingBufferIpcData<ActionGroupSharedData, 8>();
        }
        _global_data._available_data_ptr += sizeof(util::RingBufferIpcData<ActionGroupSharedData, 8>) * MAX_ENGINE_NUMBER;
    }

    _global_data._available_data_ptr = align_up<util::RingBufferIpcData<ActionGroupSharedData, 32>>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._program_list = ShmPtr<util::RingBufferIpcData<ActionGroupSharedData, 32>>(off);
        for (int i = 0; i < MAX_ENGINE_NUMBER; i++) {
            new (_global_data._available_data_ptr + i * sizeof(util::RingBufferIpcData<ActionGroupSharedData, 32>))
                util::RingBufferIpcData<ActionGroupSharedData, 32>();
        }
        _global_data._available_data_ptr += sizeof(util::RingBufferIpcData<ActionGroupSharedData, 32>) * MAX_ENGINE_NUMBER;
    }

    _global_data._available_data_ptr = align_up<util::RingBufferIpcData<PathSplineData, 64>>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._trajectory_list = ShmPtr<util::RingBufferIpcData<PathSplineData, 64>>(off);
        for (int i = 0; i < MAX_ENGINE_NUMBER; i++) {
            new (_global_data._available_data_ptr + i * sizeof(util::RingBufferIpcData<PathSplineData, 64>))
                util::RingBufferIpcData<PathSplineData, 64>();
        }
        _global_data._available_data_ptr += sizeof(util::RingBufferIpcData<PathSplineData, 64>) * MAX_ENGINE_NUMBER;
    }

    _global_data._available_data_ptr = align_up<util::RingBufferIpcData<ServoStepData, 16>>(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._micro_list = ShmPtr<util::RingBufferIpcData<ServoStepData, 16>>(off);
        new (_global_data._available_data_ptr) util::RingBufferIpcData<ServoStepData, 16>();
        _global_data._available_data_ptr += sizeof(util::RingBufferIpcData<ServoStepData, 16>);
    }

    // Initialize RingBufferIpc wrappers pointing into shared memory.
    program_action_list_ = new util::RingBufferIpc<ActionGroupSharedData, 32>(_global_data._program_list.get());
    program_action_list_->Init();
    manual_action_list_ = new util::RingBufferIpc<ActionGroupSharedData, 8>(_global_data._manual_list.get());
    manual_action_list_->Init();
    trajectory_list_ = new util::RingBufferIpc<PathSplineData, 64>(_global_data._trajectory_list.get());
    trajectory_list_->Init();
    micro_list_ = new util::RingBufferIpc<ServoStepData, 16>(_global_data._micro_list.get());
    micro_list_->Init();

    // Align to page boundary for RobotConfig
    _global_data._available_data_ptr = align_page(_global_data._available_data_ptr);
    {
        size_t off = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
        _global_data._robot_config = ShmPtr<char>(off);
    }

    static pthread_mutexattr_t mutex_attr;
    if (pthread_mutexattr_init(&mutex_attr) != 0) {
        return false;
    }
    if (pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED) != 0) {
        pthread_mutexattr_destroy(&mutex_attr);
        return false;
    }
    if (pthread_mutex_init(_global_data._global_mutex, &mutex_attr) != 0) {
        perror("pthread_mutex_init");
        pthread_mutexattr_destroy(&mutex_attr);
        return false;
    }
    pthread_mutexattr_destroy(&mutex_attr);

    _task_shared_data = &_global_data;

    return true;
}

bool CTaskFrame::RegisterTask()
{
    if (_task_shared_data == nullptr || _task_shared_data->_global_mutex == nullptr) {
        return false;
    }

    pthread_mutex_lock(_task_shared_data->_global_mutex);

    _task_id = (*_task_shared_data->_task_count)++;
    status_mgr_.NewTaskStatus(_task_name);
    if (!_tf_event_mgr.RegisterTask(_task_id, _task_name)) {
        Log_W("CTaskFrame::RegisterTask: event registry failed for %s\n", _task_name);
    }
    if (!_tf_msg_queue_mgr.RegisterQueue(_task_id, _task_name)) {
        Log_W("CTaskFrame::RegisterTask: msg registry failed for %s\n", _task_name);
    }

    pthread_mutex_unlock(_task_shared_data->_global_mutex);

    strcpy(_task_data._task_name, _task_name);

    return true;
}

bool CTaskFrame::InitTask(char * task_name)
{
    strcpy(_task_name, task_name);
    InitPhase completed = InitPhase::NONE;

    if (!OpenSharedMemory()) {
        Log_D("CTaskFrame: OpenSharedMemory failed\n");
        return CleanupAndFail(completed);
    }
    completed |= InitPhase::SHM_OPENED;

    if (_is_main_task) {
        if (!InitGlobalData()) {
            Log_E("CTaskFrame: InitGlobalData failed\n");
            return CleanupAndFail(completed);
        }
        completed |= InitPhase::GLOBAL_DATA_INITED;

        std::string urdf_path;
        std::string cfg_path;
        if (!ConfigPathResolver::Resolve(urdf_path, cfg_path)) {
            return CleanupAndFail(completed);
        }

        if (!BuildRobotConfig(urdf_path.c_str(), cfg_path.c_str())) {
            Log_E("CTaskFrame: BuildRobotConfig failed\n");
            return CleanupAndFail(completed);
        }
        _global_data._available_data_ptr += _global_data._robot_config_size;
        completed |= InitPhase::ROBOT_CONFIG_LOADED;

        if (!InitNewSubsystems()) {
            Log_E("CTaskFrame: InitNewSubsystems failed\n");
            return CleanupAndFail(completed);
        }
        completed |= InitPhase::SUBSYSTEMS_INITED;

        Log_D("Shared memory size is %d\n",
              _global_data._available_data_ptr - (char *)_global_data._shared_data_ptr);

        // Copy GlobalData into shared memory so all processes can access it
        {
            size_t gd_offset = sizeof(ShmHeader);
            GlobalData* shared_gd = reinterpret_cast<GlobalData*>(
                static_cast<char*>(_global_data._shared_data_ptr) + gd_offset);
            *shared_gd = _global_data;
        }

        status_mgr_ = CStatusManager(_global_data._task_status.get());
        data_sender_ = CDataSender(_global_data._data_sender.get());
    }
    else {
        if (_global_data._shared_data_ptr == nullptr) {
            Log_D("Shared memory is NULL!\n");
            return CleanupAndFail(completed);
        }

        // Validate header before trusting shared memory content.
        ShmHeader* header = static_cast<ShmHeader*>(_global_data._shared_data_ptr);
        if (header->magic != SHM_MAGIC || header->version != kShmLayoutVersion) {
            Log_E("CTaskFrame: Shared memory header invalid\n");
            return CleanupAndFail(completed);
        }

        // Use GlobalData from shared memory (set by main task)
        size_t gd_offset = sizeof(ShmHeader);
        GlobalData* shared_gd = reinterpret_cast<GlobalData*>(
            static_cast<char*>(_global_data._shared_data_ptr) + gd_offset);

        if (shared_gd->_global_mutex == nullptr) {
            Log_D("GlobalData not yet initialized by main task\n");
            return CleanupAndFail(completed);
        }

        // Copy shared GlobalData into a local per-process copy so that
        // absolute pointer repair does not race with other processes.
        _local_shared_data = *shared_gd;
        _task_shared_data = &_local_shared_data;

        // Fix absolute pointers: main task's mmap address may differ
        // from this process's mmap address. Rebase using offsets.
        char* orig_base = (char*)shared_gd->_shared_data_ptr;
        char* curr_base = (char*)g_shm_base;
        if (orig_base != nullptr && orig_base != curr_base) {
            if (_task_shared_data->_exit_flag)
                _task_shared_data->_exit_flag = (bool*)(curr_base +
                    ((char*)_task_shared_data->_exit_flag - orig_base));
            if (_task_shared_data->_stop_mode)
                _task_shared_data->_stop_mode = (STOP_MODE_TYPE*)(curr_base +
                    ((char*)_task_shared_data->_stop_mode - orig_base));
            if (_task_shared_data->_global_mutex)
                _task_shared_data->_global_mutex = (pthread_mutex_t*)(curr_base +
                    ((char*)_task_shared_data->_global_mutex - orig_base));
            if (_task_shared_data->_task_count)
                _task_shared_data->_task_count = (int*)(curr_base +
                    ((char*)_task_shared_data->_task_count - orig_base));
            if (_task_shared_data->_shared_data_ptr)
                _task_shared_data->_shared_data_ptr = (void*)(curr_base +
                    ((char*)_task_shared_data->_shared_data_ptr - orig_base));
            if (_task_shared_data->_available_data_ptr)
                _task_shared_data->_available_data_ptr = (char*)(curr_base +
                    ((char*)_task_shared_data->_available_data_ptr - orig_base));
        }

        completed |= InitPhase::GLOBAL_DATA_INITED;

        if (!AttachRobotConfig()) {
            Log_D("CTaskFrame: RobotConfig not yet available\n");
            return CleanupAndFail(completed);
        }

        if (!AttachNewSubsystems()) {
            Log_D("CTaskFrame: AttachNewSubsystems failed (main task may not be ready)\n");
            return CleanupAndFail(completed);
        }
        completed |= InitPhase::SUBSYSTEMS_INITED;

        // Attach RingBufferIpc wrappers to shared-memory data structs.
        program_action_list_ = new util::RingBufferIpc<ActionGroupSharedData, 32>(_task_shared_data->_program_list.get());
        program_action_list_->Attach();
        manual_action_list_ = new util::RingBufferIpc<ActionGroupSharedData, 8>(_task_shared_data->_manual_list.get());
        manual_action_list_->Attach();
        trajectory_list_ = new util::RingBufferIpc<PathSplineData, 64>(_task_shared_data->_trajectory_list.get());
        trajectory_list_->Attach();
        micro_list_ = new util::RingBufferIpc<ServoStepData, 16>(_task_shared_data->_micro_list.get());
        micro_list_->Attach();

        status_mgr_ = CStatusManager(_task_shared_data->_task_status.get());
        data_sender_ = CDataSender(_task_shared_data->_data_sender.get());
    }

    if (!RegisterTask()) {
        Log_E("CTaskFrame: RegisterTask failed\n");
        return CleanupAndFail(completed);
    }
    completed |= InitPhase::TASK_REGISTERED;

    return true;
}

bool CTaskFrame::CleanupAndFail(InitPhase completed)
{
    if (HasPhase(completed, InitPhase::SUBSYSTEMS_INITED) && _is_main_task) {
        // New subsystems currently do not expose individual cleanup.
        // Leaving shared memory unlinked below is safe because no other
        // process can attach to an unlinked object we are about to munmap.
    }

    if (HasPhase(completed, InitPhase::ROBOT_CONFIG_LOADED) && _is_main_task) {
        // RobotConfig was built into shared memory. Rewind the available pointer
        // and reset size so the shared memory layout is consistent if anything
        // tries to inspect it before unlink.
        _global_data._available_data_ptr -= _global_data._robot_config_size;
        _global_data._robot_config_size = 0;
    }

    if (HasPhase(completed, InitPhase::GLOBAL_DATA_INITED) && _is_main_task) {
        if (_global_data._global_mutex) {
            pthread_mutex_destroy(_global_data._global_mutex);
            _global_data._global_mutex = nullptr;
        }
    }

    if (HasPhase(completed, InitPhase::SHM_OPENED)) {
        if (_global_data._shared_data_ptr && _global_data._shared_data_ptr != MAP_FAILED) {
            munmap(_global_data._shared_data_ptr, SHARED_MEMORY_SIZE);
            _global_data._shared_data_ptr = nullptr;
        }
        if (_is_main_task) {
            shm_unlink(SHARED_MEMORY_NAME);
        }
    }

    _task_shared_data = nullptr;
    return false;
}

bool CTaskFrame::OpenSharedMemory()
{
    // If already mapped in this process (e.g., multiple main tasks in test), reuse
    if (_global_data._shared_data_ptr != nullptr) {
        g_shm_base = _global_data._shared_data_ptr;
        g_shm_size = SHARED_MEMORY_SIZE;
        return true;
    }

    if(_is_main_task)
    {
        // Main task always unlinks any stale shared memory and creates fresh
        shm_unlink(SHARED_MEMORY_NAME);
        int fd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            perror("shm_open");
            return false;
        }
        if (ftruncate(fd, SHARED_MEMORY_SIZE) < 0) {
            perror("ftruncate");
            close(fd);
            return false;
        }
        _global_data._shared_data_ptr = mmap(nullptr, SHARED_MEMORY_SIZE,
                                              PROT_READ | PROT_WRITE,
                                              MAP_SHARED, fd, 0);
        close(fd);
        if (_global_data._shared_data_ptr == MAP_FAILED) {
            _global_data._shared_data_ptr = nullptr;
            return false;
        }

        // Write header
        ShmHeader* header = static_cast<ShmHeader*>(_global_data._shared_data_ptr);
        header->magic = SHM_MAGIC;
        header->version = kShmLayoutVersion;
        header->size = SHARED_MEMORY_SIZE;

        g_shm_base = _global_data._shared_data_ptr;
        g_shm_size = SHARED_MEMORY_SIZE;
    }
    else
    {
        // Non-main task: open existing shared memory
        int fd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0);
        if (fd < 0) {
            perror("shm_open");
            return false;
        }
        _global_data._shared_data_ptr = mmap(nullptr, SHARED_MEMORY_SIZE,
                                              PROT_READ | PROT_WRITE,
                                              MAP_SHARED, fd, 0);
        close(fd);
        if (_global_data._shared_data_ptr == MAP_FAILED) {
            _global_data._shared_data_ptr = nullptr;
            return false;
        }

        // Verify header
        ShmHeader* header = static_cast<ShmHeader*>(_global_data._shared_data_ptr);
        if (header->magic != SHM_MAGIC || header->version != kShmLayoutVersion) {
            Log_E("Shared memory header invalid\n");
            munmap(_global_data._shared_data_ptr, SHARED_MEMORY_SIZE);
            _global_data._shared_data_ptr = nullptr;
            return false;
        }

        g_shm_base = _global_data._shared_data_ptr;
        g_shm_size = SHARED_MEMORY_SIZE;
    }
    return true;
}



void CTaskFrame::SetExitFlag(bool flag)
{
    if (_task_shared_data && _task_shared_data->_exit_flag)
        *_task_shared_data->_exit_flag = flag;
    else
        Log_D("Exit flag not available\n");
}

bool CTaskFrame::GetExitFlag()
{
    if (_task_shared_data && _task_shared_data->_exit_flag)
        return (*_task_shared_data->_exit_flag);
    else {
        Log_D("Exit flag not available, assuming true to stop loop\n");
        return true;
    }
}

STOP_MODE_TYPE CTaskFrame::GetStopMode()
{
    if (_task_shared_data && _task_shared_data->_stop_mode)
        return *_task_shared_data->_stop_mode;
    Log_D("Stop mode not available, returning STOP_FAST\n");
    return STOP_FAST;
}
void CTaskFrame::SetStopMode(STOP_MODE_TYPE stop_mode)
{
    if (_task_shared_data && _task_shared_data->_stop_mode)
        *_task_shared_data->_stop_mode = stop_mode;
    else
        Log_D("Stop mode not available\n");
}

void CTaskFrame::GetTaskData(CTaskDataBase & taskData)
{
    taskData = _task_data;
}

void CTaskFrame::SetTaskData(CTaskDataBase & taskData)
{
    _task_data = taskData;
}

TASK_STATUS_TYPE CTaskFrame::ExpectedStatusForEvent(EVENT_TYPE event) const
{
    switch (event) {
        case INIT_EVENT:           return UNCONFIGED;
        case CONFIG_EVENT:         return MOTOR_OFF;
        case MOTOR_ON_EVENT:       return READY;
        case CONTROL_ON_EVENT:     return RUNNING;
        case CONTROL_OFF_EVENT:    return READY;
        case MOTOR_OFF_EVENT:      return MOTOR_OFF;
        case ERROR_EVENT:          return ERROR;
        case RESET_EVENT:          return MOTOR_OFF;
        case EMERGENCY_STOP_EVENT: return EMERGENCY_STOP;
        case SAFE_STOP_EVENT:      return MOTOR_OFF;
        default:                   return UNINTIATED;
    }
}

bool CTaskFrame::EventRequiresLeavingMotion(EVENT_TYPE event) const
{
    return event == CONTROL_OFF_EVENT ||
           event == MOTOR_OFF_EVENT ||
           event == ERROR_EVENT ||
           event == EMERGENCY_STOP_EVENT ||
           event == SAFE_STOP_EVENT;
}

bool CTaskFrame::IsMotionStopped() const
{
    return true;
}

void CTaskFrame::GetEvent(EVENT_TYPE & event)
{
    event = EMPTY_EVENT;
    if (_task_id < 0 || _task_id >= MAX_TASK_NUMBER) return;

    // If a previous event is still pending, wait for the FSM to reach the
    // expected target status.  From MOTION_STATE we also wait for motion to
    // stop before allowing the transition.
    if (_pending_event != EMPTY_EVENT) {
        if (GetTaskStatus() == _pending_expected_status) {
            _pending_event = EMPTY_EVENT;
            _event_sync_wait = 0;
            _pending_event_dispatched = false;
        } else {
            if (_event_sync_wait >= kEventSyncTimeoutCycles) {
                Log_E("CTaskFrame::GetEvent: task %d event %d sync timeout\n",
                      _task_id, _pending_event);
                if (GetEventManager().IsWriter()) {
                    // 主任务：广播 ERROR（SPSC 规则下只有写端可广播）
                    GetEventManager().PublishBroadcast(EventType::ERROR);
                } else {
                    // 子任务：经状态管理器上报 ERROR，由主任务轮询后统一广播
                    SetTaskStatus(ERROR);
                }
                _pending_event = EMPTY_EVENT;
                _event_sync_wait = 0;
                _pending_event_dispatched = false;
                event = ERROR_EVENT;
                return;
            }
            _event_sync_wait++;

            if (!_pending_event_dispatched) {
                // From MOTION_STATE, block the transition until motion stops.
                if (GetTaskStatus() == RUNNING &&
                    EventRequiresLeavingMotion(_pending_event) &&
                    !IsMotionStopped()) {
                    Log_D("CTaskFrame::GetEvent: task %d waiting for motion stop before event %d\n",
                          _task_id, _pending_event);
                    return;
                }
                _pending_event_dispatched = true;
                event = _pending_event;
            }
            return;
        }
    }

    // No pending event: consume exactly one event from the queue (FIFO).
    Event ev;
    if (!_tf_event_mgr.ConsumeEvent(_task_id, ev)) {
        return;
    }

    _pending_event = EventTypeToLegacy(ev.type);
    _pending_expected_status = ExpectedStatusForEvent(_pending_event);
    _event_sync_wait = 0;
    _pending_event_dispatched = false;

    // Handle immediate motion-stop blocking on the first call as well.
    if (GetTaskStatus() == RUNNING &&
        EventRequiresLeavingMotion(_pending_event) &&
        !IsMotionStopped()) {
        Log_D("CTaskFrame::GetEvent: task %d waiting for motion stop before event %d\n",
              _task_id, _pending_event);
        return;
    }

    _pending_event_dispatched = true;
    event = _pending_event;
}

void CTaskFrame::SetTaskStatus(TASK_STATUS_TYPE task_status)
{
    status_mgr_.SetStatus(_task_name, task_status);
    _current_status = task_status;
}

void CTaskFrame::SetLastTaskStatus(TASK_STATUS_TYPE task_status)
{
    _last_status = task_status;
    
}
TASK_STATUS_TYPE CTaskFrame::GetTaskStatus()
{
    return _current_status;

}
TASK_STATUS_TYPE CTaskFrame::GetLastTaskStatus()
{
    return _last_status;

}

void CTaskFrame::UpdateTaskStatus()
{

    
}

CDataSender& CTaskFrame::GetDataSender()
{
    return data_sender_;
}

int CTaskFrame::InitAction()
{
    Log_D("TaskFrame InitAction\n");
    return 0;
}

int CTaskFrame::ConfigAction()
{
    Log_D("TaskFrame ConfigAction\n");
    return 0;
}

int CTaskFrame::ErrorAction()
{
    Log_D("TaskFrame ErrorAction\n");
    return 0;
}

int CTaskFrame::MotorOnAction()
{
    Log_D("TaskFrame MotorOnAction\n");
    return 0;
}
int CTaskFrame::MotorOffAction()
{
    Log_D("TaskFrame MotorOffAction\n");
    return 0;
}
int CTaskFrame::ControlOnAction()
{
    Log_D("TaskFrame ControlOnAction\n");
    return 0;
}
int CTaskFrame::ControlOffAction()
{
    Log_D("TaskFrame ControlOffAction\n");
    return 0;
}
int CTaskFrame::MainLoop()
{
    // Log_D("TaskFrame MainLoop\n");
    return 0;
}

int CTaskFrame::ResetAction()
{
    Log_D("TaskFrame ResetAction\n");
    return 0;
}

int CTaskFrame::EstopAction()
{
    Log_D("TaskFrame EstopAction\n");
    int result = ControlOffAction();
    result |= MotorOffAction();
    return result;
}

int CTaskFrame::EstopResetAction()
{
    Log_D("TaskFrame EstopResetAction\n");
    return ResetAction();
}


void CTaskFrame::ReadyStateRun()
{
    // Log_D("TaskFrame ReadyStateRun\n");
}
void CTaskFrame::MotionStateRun()
{
    // Log_D("TaskFrame MotionStateRun\n");
}

void CTaskFrame::ErrorStateRun()
{
    // Log_D("TaskFrame ErrorStateRun\n");
}

void CTaskFrame::EstopStateRun()
{
    // Log_D("TaskFrame EstopStateRun\n");
}

bool CTaskFrame::StartEmergencyPlannedStop() { return false; }
bool CTaskFrame::EmergencyPlannedStopRun() { return true; }



void CTaskFrame::DispatchEvent(EVENT_TYPE event)
{
    Log_D("TaskFrame DispatchEvent\n");
}

bool CTaskFrame::InitNewSubsystems() {
    if (_new_subsystems_inited) return true;

    size_t rt_offset = _global_data._available_data_ptr - (char*)_global_data._shared_data_ptr;
    rt_offset = (rt_offset + 63) & ~63ULL;
    _global_data._available_data_ptr = (char*)_global_data._shared_data_ptr + rt_offset;

    static TfShmRegionManager region_mgr;
    if (!region_mgr.Init(_global_data._shared_data_ptr, SHARED_MEMORY_SIZE, rt_offset)) {
        Log_E("CTaskFrame: Failed to init region manager\n");
        return false;
    }

    void* evt_base = region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    size_t evt_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT);
    if (!_tf_event_mgr.Init(evt_base, evt_size, MAX_TASK_NUMBER)) {
        Log_E("CTaskFrame: Failed to init event manager\n");
        return false;
    }

    void* mq_base = region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
    size_t mq_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE);
    if (!_tf_msg_queue_mgr.Init(mq_base, mq_size, MAX_TASK_NUMBER)) {
        Log_E("CTaskFrame: Failed to init msg queue manager\n");
        return false;
    }

    void* fb_base = region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
    size_t fb_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK);
    if (!_tf_feedback_mgr.Init(fb_base, fb_size)) {
        Log_E("CTaskFrame: Failed to init feedback manager\n");
        return false;
    }

    size_t total_new_size = TfShmRegionManager::ComputeTotalRegionsSize();
    _global_data._available_data_ptr = (char*)_global_data._shared_data_ptr
                                       + rt_offset + sizeof(TfShmRegionTable)
                                       + total_new_size;

    _new_subsystems_inited = true;
    return true;
}

bool CTaskFrame::AttachNewSubsystems() {
    if (_new_subsystems_inited) return true;

    static TfShmRegionManager region_mgr;
    if (!region_mgr.Attach(_global_data._shared_data_ptr, SHARED_MEMORY_SIZE)) {
        return false;
    }

    void* evt_base = region_mgr.GetRegionBase(TfShmRegionId::EVENT);
    size_t evt_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT);
    if (!_tf_event_mgr.Attach(evt_base, evt_size)) {
        return false;
    }

    void* mq_base = region_mgr.GetRegionBase(TfShmRegionId::MSG_QUEUE);
    size_t mq_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::MSG_QUEUE);
    if (!_tf_msg_queue_mgr.Attach(mq_base, mq_size)) {
        return false;
    }

    void* fb_base = region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK);
    size_t fb_size = region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK);
    if (!_tf_feedback_mgr.Attach(fb_base, fb_size)) {
        return false;
    }

    _new_subsystems_inited = true;
    return true;
}

bool CTaskFrame::BuildRobotConfig(const char* urdf_path, const char* cfg_path) {
    // 1. Load config using RobotConfigBuilder
    robot_config::RobotConfigBuilder builder;
    if (!builder.Init()) {
        Log_E("BuildRobotConfig: builder.Init() failed\n");
        return false;
    }
    if (!builder.LoadDriveParams(cfg_path)) {
        Log_E("BuildRobotConfig: LoadDriveParams failed: %s\n", cfg_path);
        return false;
    }
    if (!builder.LoadUrdfKinematics(urdf_path)) {
        Log_E("BuildRobotConfig: LoadUrdfKinematics failed: %s\n", urdf_path);
        return false;
    }

    // 2. Determine required size and reserve page-aligned space
    size_t required_size = builder.GetSerializedSize();
    if (required_size == 0) {
        Log_E("BuildRobotConfig: GetSerializedSize returned 0\n");
        return false;
    }
    size_t reserved_size = ((required_size + robot_config::kShmPageSize - 1)
                            / robot_config::kShmPageSize) * robot_config::kShmPageSize;

    // Guard against shared memory overflow (ROBOT_GLOBAL_DATA is 100 MB)
    size_t shm_used = _global_data._robot_config.offset() + reserved_size;
    if (shm_used > SHARED_MEMORY_SIZE) {
        Log_E("BuildRobotConfig: required size %zu exceeds shared memory (used %zu of %d)\n",
              reserved_size, shm_used, SHARED_MEMORY_SIZE);
        return false;
    }

    // 3. Serialize directly into shared memory
    char* dest = _global_data._robot_config.get();
    if (!dest) {
        Log_E("BuildRobotConfig: _robot_config offset not set\n");
        return false;
    }
    // BuildToMemory requires page-aligned output buffer; InitTask places the
    // RobotConfig region on a page boundary.
    if ((reinterpret_cast<uintptr_t>(dest) % robot_config::kShmPageSize) != 0) {
        Log_E("BuildRobotConfig: destination not page-aligned (addr=%p)\n", static_cast<void*>(dest));
        return false;
    }

    size_t written = 0;
    if (!builder.BuildToMemory(dest, reserved_size, &written)) {
        Log_E("BuildRobotConfig: BuildToMemory failed\n");
        return false;
    }
    _global_data._robot_config_size = written;

    // 4. Attach the store to the built memory so callers can query it
    if (_robot_config_store.Attach(dest, written) != robot_config::ConfigError::OK) {
        Log_E("BuildRobotConfig: RobotConfigStore attach failed\n");
        return false;
    }

    Log_I("RobotConfig built: %zu bytes serialized to shared memory at offset %zu (reserved %zu)\n",
          written, _global_data._robot_config.offset(), reserved_size);
    return true;
}

bool CTaskFrame::AttachRobotConfig() {
    // In normal runtime AttachRobotConfig() is called after InitTask(), which
    // sets _task_shared_data. Allow tests that call AttachRobotConfig() without
    // InitTask() to fall back to the global _global_data.
    GlobalData* gd = _task_shared_data ? _task_shared_data : &_global_data;
    char* base = gd->_robot_config.get();
    if (!base) {
        Log_E("AttachRobotConfig: _robot_config not set by main task\n");
        return false;
    }
    size_t region_size = gd->_robot_config_size;
    if (region_size == 0) {
        Log_E("AttachRobotConfig: _robot_config_size is 0\n");
        return false;
    }

    // In single-process tests the singleton store may already be attached.
    // If it is, and the base/size match, treat as success.
    if (_robot_config_store.IsAttached()) {
        return true;
    }

    if (_robot_config_store.Attach(base, region_size) != robot_config::ConfigError::OK) {
        Log_E("AttachRobotConfig: RobotConfigStore attach failed\n");
        return false;
    }
    const auto* header = _robot_config_store.GetHeader();
    if (header) {
        Log_I("RobotConfig attached: version=%u, units=%u, total_size=%llu\n",
              header->version, header->num_units,
              static_cast<unsigned long long>(header->total_size));
    } else {
        Log_W("AttachRobotConfig: store attached but header is null\n");
    }
    return true;
}

void* CTaskFrame::GetRobotConfigBase() const {
    // Kept non-const void* for backward compatibility with existing callers/tests.
    return const_cast<void*>(static_cast<const void*>(_robot_config_store.GetHeader()));
}

bool CTaskFrame::IsRobotConfigReady() const {
    return _robot_config_store.IsAttached();
}

bool CTaskFrame::GetDynGroupNumber(int unit_type, int dyngrp_model,
                                   int32_t* out_dyngrp_no) const {
    if (out_dyngrp_no == nullptr) {
        return false;
    }
    if (!_robot_config_store.IsAttached()) {
        Log_E("GetDynGroupNumber: robot config not attached\n");
        return false;
    }
    return _robot_config_store.GetDynGroupNumber(unit_type, dyngrp_model,
                                                 out_dyngrp_no);
}

bool CTaskFrame::GetPowerRegainLimit(int unit_type, int dyngrp_model,
                                     double* out_limit) const {
    if (out_limit == nullptr) {
        return false;
    }
    if (!_robot_config_store.IsAttached()) {
        Log_E("GetPowerRegainLimit: robot config not attached\n");
        return false;
    }
    return _robot_config_store.GetPowerRegainLimit(unit_type, dyngrp_model,
                                                   out_limit);
}

const robot_config::UnitConfig* CTaskFrame::GetUnitConfig(int unit_type) const {
    if (!_robot_config_store.IsAttached()) {
        return nullptr;
    }
    const auto* typed_unit = _robot_config_store.GetUnit(
        static_cast<robot_config::RobotSocUnitType>(unit_type));
    return reinterpret_cast<const robot_config::UnitConfig*>(typed_unit);
}

const robot_config::RobotTopology* CTaskFrame::GetRobotTopology() const {
    if (!_robot_config_store.IsAttached()) {
        return nullptr;
    }
    return _robot_config_store.GetRobotTopology();
}

const char* CTaskFrame::GetKinematicModelName(int unit_type) const {
    const auto* unit = GetUnitConfig(unit_type);
    return unit ? unit->kinematic_model_name : nullptr;
}

const char* CTaskFrame::GetRootLink(int unit_type) const {
    const auto* unit = GetUnitConfig(unit_type);
    return unit ? unit->root_link : nullptr;
}

const char* CTaskFrame::GetTipLink(int unit_type) const {
    const auto* unit = GetUnitConfig(unit_type);
    return unit ? unit->tip_link : nullptr;
}

CJointPos CTaskFrame::MakeJointPos(double* external_data) const {
    if (external_data == nullptr || !_robot_config_store.IsAttached()) {
        return CJointPos(nullptr, nullptr, nullptr);
    }
    // Typed store returns HumanoidGlobalJointMap/DualArmGlobalJointMap, which are
    // layout-compatible with robot_config::GlobalJointMap. Same for params.
    const auto* typed_map = _robot_config_store.GetGlobalJointMap();
    const auto* typed_params = _robot_config_store.GetGlobalJointParams();
    const auto* map = reinterpret_cast<const robot_config::GlobalJointMap*>(typed_map);
    const auto* params = reinterpret_cast<const robot_config::GlobalJointParams*>(typed_params);
    return CJointPos(external_data, map, params);
}