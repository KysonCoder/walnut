#include <cstring>
#include <iostream>
#include "worker_base.hpp"
#include "task_frame.hpp"

class TestWorker : public WorkerBase {
public:
    TestWorker(const std::string& name, int ctrl_fd)
        : WorkerBase(name, ctrl_fd) {}

protected:
    CTaskFrame* create_task() override {
        CTaskFrame* task = new CTaskFrame();
        task->SetMainTask();
        return task;
    }
};

int main(int argc, char* argv[]) {
    std::string name = "test_worker";
    int ctrl_fd = 3;
    int priority = 80;
    int cpu = 2;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--name=", 7) == 0) name = argv[i] + 7;
        else if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) ctrl_fd = atoi(argv[i] + 10);
        else if (strncmp(argv[i], "--priority=", 11) == 0) priority = atoi(argv[i] + 11);
        else if (strncmp(argv[i], "--cpu=", 6) == 0) cpu = atoi(argv[i] + 6);
    }

    TestWorker worker(name, ctrl_fd);
    worker.setup_realtime(priority, cpu);

    if (!worker.init()) {
        std::cerr << "TestWorker init failed\n";
        return 1;
    }

    worker.run();
    return 0;
}
