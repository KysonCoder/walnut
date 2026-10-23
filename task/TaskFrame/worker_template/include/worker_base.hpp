#pragma once
#include <string>
#include <thread>
#include <atomic>
#include "task_frame.hpp"
#include "ipc_protocol.hpp"

class WorkerBase {
public:
    WorkerBase(const std::string& name, int ctrl_fd);
    virtual ~WorkerBase();

    virtual bool init();
    void run();
    void stop();

protected:
    virtual CTaskFrame* create_task() = 0;

    std::string _name;
    CTaskFrame* _task = nullptr;

public:
    void setup_realtime(int priority, int cpu);

private:
    void heartbeat_thread();
    void handle_ctrl_message();
    void map_shared_memory();

    int _ctrl_fd = -1;
    std::atomic<bool> _running{false};
    std::thread _heartbeat_thread;
};
