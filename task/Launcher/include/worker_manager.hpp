#pragma once
#include <string>
#include <vector>
#include <chrono>
#include "config_parser.hpp"
#include "ipc_protocol.hpp"

struct WorkerProcess {
    WorkerConfig config;
    pid_t pid = -1;
    int ctrl_socket = -1;
    int restart_count = 0;
    std::chrono::steady_clock::time_point last_heartbeat;
    bool running = false;
};

class WorkerManager {
public:
    bool start_all(const LauncherConfig& config);
    void stop_all();
    void check_and_restart();
    void handle_heartbeat(pid_t pid);
    void handle_worker_exit(pid_t pid, int status);
    const std::vector<WorkerProcess>& workers() const { return _workers; }

private:
    bool start_worker(WorkerProcess& worker);
    std::vector<WorkerProcess> _workers;
};
