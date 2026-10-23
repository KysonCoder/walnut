#include "state_master.hpp"
#include "worker_base.hpp"
#include "hmi_server.hpp"
#include "logger.h"
#include <unistd.h>
#include <cstring>
#include <memory>

class StateMasterTask : public CStateMasterTask {
public:
    int MainLoop() override {
        // 主任务事件泵：直接消费自身事件环（HTTP 线程是唯一生产者，SPSC 成立）。
        // 不能用 CTaskFrame::GetEvent() 的 pending 机制：主任务状态字要等子任务
        // 同步后才落（UpdateTaskStatus，80 周期窗口），pending 的 3 周期超时会把
        // 正常同步误判为失败并广播 ERROR。子任务同步由 UpdateTaskStatus 负责。
        Event ev;
        if (GetEventManager().ConsumeEvent(GetTaskId(), ev)) {
            DispatchEvent(EventTypeToLegacy(ev.type));
        }
        UpdateTaskStatus();
        BaseFSM<CStateMasterTask>::current_state_ptr->run();
        usleep(64000);
        return 0;
    }
};

class StateMasterWorker : public WorkerBase {
public:
    StateMasterWorker(const std::string& name, int ctrl_fd)
        : WorkerBase(name, ctrl_fd) {}

protected:
    CTaskFrame* create_task() override {
        return new StateMasterTask();
    }

public:
    bool init() override {
        _task = create_task();
        if (!_task) {
            Log_E("StateMasterWorker: failed to create task\n");
            return false;
        }

        char name_buf[32];
        strncpy(name_buf, _name.c_str(), sizeof(name_buf) - 1);
        name_buf[sizeof(name_buf) - 1] = '\0';

        _task->SetMainTask();

        if (!_task->OpenSharedMemory()) {
            Log_E("StateMasterWorker: OpenSharedMemory failed\n");
            return false;
        }

        if (!_task->InitTask(name_buf)) {
            Log_E("StateMasterWorker: InitTask failed\n");
            return false;
        }

        return true;
    }

    CStateMasterTask* state_master_task() {
        return static_cast<CStateMasterTask*>(_task);
    }
};

int main(int argc, char* argv[]) {
    std::string name = "state_master";
    int ctrl_fd = -1;
    int priority = 80;
    int cpu = 2;
    int hmi_port = 8080;
    bool hmi_enabled = true;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--name=", 7) == 0) {
            name = argv[i] + 7;
        } else if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) {
            ctrl_fd = atoi(argv[i] + 10);
        } else if (strncmp(argv[i], "--priority=", 11) == 0) {
            priority = atoi(argv[i] + 11);
        } else if (strncmp(argv[i], "--cpu=", 6) == 0) {
            cpu = atoi(argv[i] + 6);
        } else if (strncmp(argv[i], "--hmi-port=", 11) == 0) {
            hmi_port = atoi(argv[i] + 11);
        } else if (strcmp(argv[i], "--no-hmi") == 0) {
            hmi_enabled = false;
        }
    }

    StateMasterWorker worker(name, ctrl_fd);
    worker.setup_realtime(priority, cpu);

    if (!worker.init()) {
        Log_E("StateMaster: init failed\n");
        return 1;
    }

    // HMI HTTP 服务：BindTask 后启动；启动失败只降级不阻塞主任务
    std::unique_ptr<state_master_hmi::CHmiInterface> hmi_if;
    std::unique_ptr<state_master_hmi::HmiServer> hmi_server;
    if (hmi_enabled) {
        hmi_if = std::make_unique<state_master_hmi::CHmiInterface>();
        if (hmi_if->BindTask(worker.state_master_task())) {
            hmi_server = std::make_unique<state_master_hmi::HmiServer>(hmi_if.get(), hmi_port);
            if (!hmi_server->Start()) {
                Log_E("StateMaster: HMI server start failed (port %d), continuing without HMI\n",
                      hmi_port);
                hmi_server.reset();
            } else {
                Log_D("StateMaster: HMI server listening on port %d\n", hmi_port);
            }
        }
    }

    worker.run();

    hmi_server.reset();  // 先停 HTTP 线程再退出
    return 0;
}
