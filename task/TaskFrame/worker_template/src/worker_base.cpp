#include "worker_base.hpp"
#include <sched.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "utilbase/logger.h"
#include "shm_header.hpp"

WorkerBase::WorkerBase(const std::string& name, int ctrl_fd)
    : _name(name), _ctrl_fd(ctrl_fd) {}

WorkerBase::~WorkerBase() {
    stop();
}

void WorkerBase::setup_realtime(int priority, int cpu) {
    struct sched_param param = {.sched_priority = priority};
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        // SCHED_FIFO requires root; fallback to normal scheduling
        Log_W("SCHED_FIFO not available (need root), using default scheduling\n");
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        perror("sched_setaffinity");
    }
}

void WorkerBase::map_shared_memory() {
    // Wait for shared memory to be created by launcher (with timeout)
    int fd = -1;
    int retries = 50;
    while (fd < 0 && retries > 0) {
        fd = shm_open("/ROBOT_GLOBAL_DATA", O_RDWR, 0);
        if (fd < 0) {
            usleep(10000);  // 10ms
            retries--;
        }
    }
    if (fd < 0) {
        perror("shm_open");
        return;
    }

    void* base = mmap(nullptr, 100 * 1024 * 1024,
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (base == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // Wait for main task to initialize the shared memory header
    ShmHeader* header = static_cast<ShmHeader*>(base);
    retries = 200;  // 2 seconds max wait
    while (header->magic != SHM_MAGIC && retries > 0) {
        usleep(10000);
        retries--;
    }
    if (header->magic != SHM_MAGIC) {
        std::cerr << "Shared memory header not initialized (timeout)\n";
        munmap(base, 100 * 1024 * 1024);
        return;
    }

    g_shm_base = base;
    g_shm_size = 100 * 1024 * 1024;
}

bool WorkerBase::init() {
    _task = create_task();
    if (!_task) {
        Log_E("Failed to create task\n");
        return false;
    }

    if (_task->IsMainTask()) {
        // Main task: create shared memory (do not wait)
        if (!_task->OpenSharedMemory()) {
            Log_E("Failed to open shared memory\n");
            return false;
        }
    } else {
        // Non-main task: attach to existing shared memory
        map_shared_memory();
        if (g_shm_base == nullptr) {
            Log_E("Failed to map shared memory\n");
            return false;
        }
    }

    char name_buf[32];
    strncpy(name_buf, _name.c_str(), sizeof(name_buf) - 1);
    name_buf[sizeof(name_buf) - 1] = '\0';

    // For non-main tasks, there is a race condition where InitTask() may
    // be called before the main task has finished initializing GlobalData.
    // Retry with backoff until it succeeds or we timeout.
    int retries = 50;
    while (retries > 0) {
        if (_task->InitTask(name_buf)) {
            return true;
        }
        usleep(50000);  // 50ms
        retries--;
    }

    Log_E("InitTask failed after retries\n");
    return false;
}

void WorkerBase::heartbeat_thread() {
    while (_running) {
        ipc::CtrlMessage msg = {};
        msg.type = ipc::CtrlMsgType::WORKER_HEARTBEAT;
        msg.pid = getpid();
        strncpy(msg.task_name, _name.c_str(), sizeof(msg.task_name) - 1);
        ipc::send_ctrl_message(_ctrl_fd, msg);

        usleep(100000);  // 100ms
    }
}

void WorkerBase::handle_ctrl_message() {
    ipc::CtrlMessage msg = {};
    ssize_t ret = ipc::recv_ctrl_message(_ctrl_fd, msg, 0);
    if (ret == sizeof(msg)) {
        if (msg.type == ipc::CtrlMsgType::LAUNCHER_STOP) {
            _running = false;
            if (_task) _task->SetExitFlag(true);
        }
    }
}

void WorkerBase::run() {
    _running = true;

    // Send registration
    ipc::CtrlMessage reg = {};
    reg.type = ipc::CtrlMsgType::WORKER_REGISTER;
    reg.pid = getpid();
    strncpy(reg.task_name, _name.c_str(), sizeof(reg.task_name) - 1);
    ipc::send_ctrl_message(_ctrl_fd, reg);

    // Start heartbeat thread
    _heartbeat_thread = std::thread(&WorkerBase::heartbeat_thread, this);

    // Main loop
    while (_running) {
        if (_task) {
            _task->MainLoop();
        }
        handle_ctrl_message();
    }
}

void WorkerBase::stop() {
    _running = false;
    if (_heartbeat_thread.joinable()) {
        _heartbeat_thread.join();
    }
    if (_ctrl_fd >= 0) {
        close(_ctrl_fd);
        _ctrl_fd = -1;
    }
}
