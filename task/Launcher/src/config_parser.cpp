#include "config_parser.hpp"
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

bool parse_config(const std::string& filepath, LauncherConfig& config) {
    try {
        YAML::Node root = YAML::LoadFile(filepath);

        if (root["system"]) {
            auto sys = root["system"];
            if (sys["shm_name"]) config.system.shm_name = sys["shm_name"].as<std::string>();
            if (sys["shm_size"]) config.system.shm_size = sys["shm_size"].as<size_t>();
            if (sys["watchdog_interval_ms"]) config.system.watchdog_interval_ms = sys["watchdog_interval_ms"].as<int>();
            if (sys["heartbeat_timeout_ms"]) config.system.heartbeat_timeout_ms = sys["heartbeat_timeout_ms"].as<int>();
        }

        if (root["tasks"]) {
            for (const auto& task : root["tasks"]) {
                WorkerConfig wc;
                wc.name = task["name"].as<std::string>();
                wc.executable = task["executable"].as<std::string>();
                if (task["priority"]) wc.priority = task["priority"].as<int>();
                if (task["cpu"]) wc.cpu = task["cpu"].as<int>();
                if (task["autorestart"]) wc.autorestart = task["autorestart"].as<bool>();
                if (task["max_restarts"]) wc.max_restarts = task["max_restarts"].as<int>();
                config.workers.push_back(wc);
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse config: " << e.what() << "\n";
        return false;
    }
}

void load_default_config(LauncherConfig& config) {
    config.system.shm_name = "/ROBOT_GLOBAL_DATA";
    config.system.shm_size = 100 * 1024 * 1024;
    config.system.watchdog_interval_ms = 100;
    config.system.heartbeat_timeout_ms = 500;

    // StateMaster must start first (main task, creates shared memory)
    WorkerConfig state_master;
    state_master.name = "state_master";
    state_master.executable = "./state_master.elf";
    state_master.priority = 80;
    state_master.cpu = 2;
    state_master.autorestart = true;
    state_master.max_restarts = 3;
    config.workers.push_back(state_master);

    WorkerConfig planning;
    planning.name = "planning_task";
    planning.executable = "./planning_task.elf";
    planning.priority = 95;
    planning.cpu = 3;
    planning.autorestart = true;
    planning.max_restarts = 3;
    config.workers.push_back(planning);

    WorkerConfig servo;
    servo.name = "servo_task";
    servo.executable = "./servo_task.elf";
    servo.priority = 96;
    servo.cpu = 3;
    servo.autorestart = true;
    servo.max_restarts = 3;
    config.workers.push_back(servo);

    WorkerConfig fieldbus;
    fieldbus.name = "fieldbus_task";
    fieldbus.executable = "./fieldbus_task.elf";
    fieldbus.priority = 98;
    fieldbus.cpu = 4;
    fieldbus.autorestart = true;
    fieldbus.max_restarts = 3;
    config.workers.push_back(fieldbus);
}
