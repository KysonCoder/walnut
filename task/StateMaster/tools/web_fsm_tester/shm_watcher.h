/** @file shm_watcher.h
 *  @brief 只读 attach /ROBOT_GLOBAL_DATA，读 4 进程状态字（搬自 full_app_state_machine_test）
 */
#ifndef WEB_FSM_TESTER_SHM_WATCHER_H_
#define WEB_FSM_TESTER_SHM_WATCHER_H_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "shm_header.hpp"
#include "shm_ptr.hpp"
#include "status_manager.hpp"
#include "task_frame.hpp"   // GlobalData / SHARED_MEMORY_NAME

namespace web_fsm_tester {

constexpr int kWorkerCount = 4;
constexpr const char* kTaskNames[kWorkerCount] = {
    "state_master", "planning_task", "servo_task", "fieldbus_task"};

struct TaskStatusInfo {
    std::string name;
    int status;   // -1 = 未注册/不可读
};

class ShmStatusWatcher {
public:
    ~ShmStatusWatcher() { Detach(); }

    bool Attach();              // 幂等；shm 未就绪返回 false（调用方轮询）
    void Detach();
    bool StatusTableReady();    // _task_status 偏移非 0 且 4 个任务名全部注册
    bool GetStatusByName(const char* name, int* out);
    std::vector<TaskStatusInfo> ReadAll();
    // abort 非空且返回 true 时提前中止等待（用于序列中止）
    bool WaitForAllTasks(int expected, int timeout_ms,
                         const std::function<bool()>& abort = nullptr);
    std::string Snapshot();

private:
    StatusManagerData* StatusData();
    void* base_ = nullptr;
    size_t size_ = 0;
    GlobalData* global_ = nullptr;
};

}  // namespace web_fsm_tester

#endif  // WEB_FSM_TESTER_SHM_WATCHER_H_
