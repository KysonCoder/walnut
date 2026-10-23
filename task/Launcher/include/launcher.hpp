#pragma once
#include <string>
#include <cstdint>
#include "config_parser.hpp"
#include "worker_manager.hpp"

class Launcher {
public:
    bool init(const std::string& config_path);
    bool init_with_default();
    bool start_all_workers();
    void stop_all_workers();
    void run_once();
    bool is_running() const { return _running; }
    void stop() { _running = false; }

    // 启动前清理残留 shm。Launcher 启动时没有任何任务在跑，现存的同名
    // shm 必为上次（可能以 root 运行或崩溃）的残留。/dev/shm 是 sticky
    // 目录，root 残留文件只有 root 能删——清理失败时给出提示并返回 false。
    static bool CleanupStaleShm(const std::string& shm_name);

private:
    bool attach_shared_memory();
    // 关停前优雅下电：经 HMI 发 msg_motor_off 并等状态同步（SPSC 规则下
    // Launcher 不是事件环写者，不能自己 PublishBroadcast）
    void graceful_motor_off();
    bool try_attach_shared_memory_once();
    int  read_all_task_status(int* out, int max_count);
    bool any_task_ready_or_running();
    bool all_tasks_motor_off_or_error();
    void handle_signals();
    void reap_workers();
    void check_heartbeats();

    bool _running = true;
    bool _crash_error_notified = false;   // 崩溃的 ERROR 通知每次崩溃 episode 只发一次
    LauncherConfig _config;
    WorkerManager _worker_mgr;
    void* _shm_base = nullptr;
};
