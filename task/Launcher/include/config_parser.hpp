#pragma once
#include <string>
#include <vector>

struct WorkerConfig {
    std::string name;
    std::string executable;
    int priority = 80;
    int cpu = 0;
    bool autorestart = true;
    int max_restarts = 5;
};

struct SystemConfig {
    std::string shm_name = "/ROBOT_GLOBAL_DATA";
    size_t shm_size = 100 * 1024 * 1024;
    int watchdog_interval_ms = 100;
    int heartbeat_timeout_ms = 500;
};

struct LauncherConfig {
    SystemConfig system;
    std::vector<WorkerConfig> workers;
};

bool parse_config(const std::string& filepath, LauncherConfig& config);
void load_default_config(LauncherConfig& config);
