#include "worker_base.hpp"
#include "task_frame.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>

class PlanningTask : public CTaskFrame {
public:
    int MainLoop() override {
        // 子进程（exec 出的真实进程）回归护栏：子任务路径下这些访问器
        // 必须非空。2026-08-26 修复前，子任务里 _global_data._program_list
        // 未填充导致 GetProgramActionList 返回 nullptr——fork 型测试因内存
        // 继承无法暴露，只有 exec 型 worker 能覆盖。返回 nullptr 即退出 42，
        // 触发 launcher 重启耗尽后进程消失，test_four_process 转红。
        if (GetProgramActionList() == nullptr ||
            GetManualActionList() == nullptr ||
            GetTrajectoryList() == nullptr) {
            std::cerr << "PlanningTask child-path accessor returned null\n";
            _exit(42);
        }
        usleep(10000);
        return 0;
    }
};

class PlanningWorker : public WorkerBase {
public:
    PlanningWorker(const std::string& name, int ctrl_fd)
        : WorkerBase(name, ctrl_fd) {}

protected:
    CTaskFrame* create_task() override {
        return new PlanningTask();
    }
};

int main(int argc, char* argv[]) {
    std::string name = "planning";
    int ctrl_fd = 3;
    int priority = 80;
    int cpu = 2;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--name=", 7) == 0) name = argv[i] + 7;
        else if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) ctrl_fd = atoi(argv[i] + 10);
        else if (strncmp(argv[i], "--priority=", 11) == 0) priority = atoi(argv[i] + 11);
        else if (strncmp(argv[i], "--cpu=", 6) == 0) cpu = atoi(argv[i] + 6);
    }

    PlanningWorker worker(name, ctrl_fd);
    worker.setup_realtime(priority, cpu);

    if (!worker.init()) {
        std::cerr << "Planning init failed\n";
        return 1;
    }

    worker.run();
    return 0;
}
