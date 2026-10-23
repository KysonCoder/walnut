/**
 * @file planning_task_main.cpp
 * @brief Planning task process entry point
 */

#include "worker_base.hpp"
#include "task_frame.hpp"
#include "planning_task.hpp"
#include <cstring>
#include <iostream>

class PlanningWorker : public WorkerBase {
public:
    PlanningWorker(const std::string& name, int ctrl_fd)
        : WorkerBase(name, ctrl_fd) {}

protected:
    CTaskFrame* create_task() override {
        return new CPlanningTask();
    }
};

int main(int argc, char* argv[]) {
    std::string name = "planning_task";
    int ctrl_fd = -1;
    int priority = 95;
    int cpu = 3;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--name=", 7) == 0) {
            name = argv[i] + 7;
        } else if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) {
            ctrl_fd = atoi(argv[i] + 10);
        } else if (strncmp(argv[i], "--priority=", 11) == 0) {
            priority = atoi(argv[i] + 11);
        } else if (strncmp(argv[i], "--cpu=", 6) == 0) {
            cpu = atoi(argv[i] + 6);
        }
    }

    PlanningWorker worker(name, ctrl_fd);
    worker.setup_realtime(priority, cpu);

    if (!worker.init()) {
        Log_E("PlanningTask: init failed\n");
        return 1;
    }

    worker.run();
    return 0;
}
