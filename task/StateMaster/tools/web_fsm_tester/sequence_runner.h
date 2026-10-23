/** @file sequence_runner.h
 *  @brief 一键完整序列：后台线程按步进表下发命令并等待 4 进程同步
 */
#ifndef WEB_FSM_TESTER_SEQUENCE_RUNNER_H_
#define WEB_FSM_TESTER_SEQUENCE_RUNNER_H_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "api_utils.h"     // SeqView
#include "hmi_client.h"
#include "shm_watcher.h"

namespace web_fsm_tester {

struct SeqStep {
    const char* msg;
    int expected_status;
    int timeout_ms;
    const char* label;
};

// 步进表（与 full_app_state_machine_test 的 GoldenPath 一致）
const std::vector<SeqStep>& SequenceSteps();

class SequenceRunner {
public:
    ~SequenceRunner() { Shutdown(); }

    bool Start(ShmStatusWatcher* watcher, HmiClient* hmi);  // 已在运行返回 false
    void RequestStop();
    void Shutdown();          // RequestStop + join（程序退出时调）
    bool IsActive() const { return active_.load(); }
    SeqView View();           // 给 /api/status 用的快照

private:
    void Run();
    void SetLabel(const std::string& s);
    void SetError(const std::string& s);

    ShmStatusWatcher* watcher_ = nullptr;
    HmiClient* hmi_ = nullptr;
    std::thread thread_;
    std::atomic<bool> active_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int> step_{0};
    mutable std::mutex mu_;
    std::string label_;
    std::string error_;
    bool finished_ = false;
    bool success_ = false;
};

}  // namespace web_fsm_tester

#endif  // WEB_FSM_TESTER_SEQUENCE_RUNNER_H_
