#include "worker_manager.hpp"
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include "task_frame.hpp"
#include "shm_header.hpp"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

bool WorkerManager::start_all(const LauncherConfig& config) {
    for (const auto& wc : config.workers) {
        WorkerProcess wp;
        wp.config = wc;
        _workers.push_back(wp);
    }

    // Start state_master first (main task) — it must finish InitTask()
    // and initialize GlobalData before other workers can attach.
    WorkerProcess* state_master = nullptr;
    for (auto& worker : _workers) {
        if (worker.config.name == "state_master") {
            state_master = &worker;
            if (!start_worker(worker)) {
                std::cerr << "Failed to start state_master\n";
                return false;
            }
            // Poll for registration/heartbeat while main task inits.
            // Do NOT sleep here — that blocks the heartbeat watchdog.
            bool registered = false;
            int waited = 0;
            while (waited < 3000) {
                ipc::CtrlMessage msg;
                ssize_t ret = ipc::recv_ctrl_message(worker.ctrl_socket, msg, 50);
                if (ret == sizeof(msg)) {
                    if (msg.type == ipc::CtrlMsgType::WORKER_REGISTER) {
                        std::cout << "Main task registered: "
                                  << msg.task_name << " (pid=" << msg.pid << ")\n";
                        handle_heartbeat(msg.pid);
                        registered = true;
                        break;
                    }
                    if (msg.type == ipc::CtrlMsgType::WORKER_HEARTBEAT) {
                        handle_heartbeat(msg.pid);
                    }
                }
                usleep(10000);
                waited += 10;
            }

            if (registered) {
                // Poll shared memory for RobotConfig ready
                int shm_fd = shm_open("/ROBOT_GLOBAL_DATA", O_RDONLY, 0);
                if (shm_fd >= 0) {
                    struct stat st;
                    if (fstat(shm_fd, &st) == 0) {
                        size_t shm_size = st.st_size;
                        if (shm_size >= sizeof(ShmHeader) + sizeof(GlobalData)) {
                            void* shm_base = mmap(nullptr, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
                            if (shm_base != MAP_FAILED) {
                                // Verify TF-SHM magic
                                ShmHeader* tf_header = static_cast<ShmHeader*>(shm_base);
                                if (tf_header->magic == SHM_MAGIC) {
                                    GlobalData* gd = reinterpret_cast<GlobalData*>(
                                        static_cast<char*>(shm_base) + sizeof(ShmHeader));
                                    int rc_waited = 0;
                                    while (rc_waited < 10000) {  // 10 second timeout
                                        if (gd->_robot_config.offset() != 0) {
                                            std::cout << "RobotConfig ready after "
                                                      << rc_waited << "ms\n";
                                            break;
                                        }
                                        usleep(10000);
                                        rc_waited += 10;
                                    }
                                    if (rc_waited >= 10000) {
                                        std::cerr << "Timeout waiting for RobotConfig\n";
                                    }
                                }
                                munmap(shm_base, shm_size);
                            }
                        }
                    }
                    close(shm_fd);
                }
            }
            break;
        }
    }

    for (auto& worker : _workers) {
        if (&worker == state_master) continue;
        if (!start_worker(worker)) {
            std::cerr << "Failed to start worker: " << worker.config.name << "\n";
            return false;
        }
    }
    return true;
}

bool WorkerManager::start_worker(WorkerProcess& worker) {
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sv) < 0) {
        perror("socketpair");
        return false;
    }

    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        close(sv[0]);
        close(sv[1]);
        return false;
    }

    if (pid == 0) {
        // Child process
        close(sv[0]);
        dup2(sv[1], 3);
        close(sv[1]);

        char name_arg[64];
        char prio_arg[32];
        char cpu_arg[32];
        char ctrl_fd_arg[32];
        snprintf(name_arg, sizeof(name_arg), "--name=%s", worker.config.name.c_str());
        snprintf(prio_arg, sizeof(prio_arg), "--priority=%d", worker.config.priority);
        snprintf(cpu_arg, sizeof(cpu_arg), "--cpu=%d", worker.config.cpu);
        snprintf(ctrl_fd_arg, sizeof(ctrl_fd_arg), "--ctrl-fd=3");

        char* args[] = {
            const_cast<char*>(worker.config.executable.c_str()),
            name_arg, prio_arg, cpu_arg, ctrl_fd_arg,
            nullptr
        };

        execv(worker.config.executable.c_str(), args);
        perror("execv");
        _exit(1);
    }

    close(sv[1]);
    worker.pid = pid;
    worker.ctrl_socket = sv[0];
    worker.running = true;
    worker.last_heartbeat = std::chrono::steady_clock::now();
    worker.restart_count++;
    return true;
}

void WorkerManager::stop_all() {
    for (auto& worker : _workers) {
        if (worker.running && worker.pid > 0) {
            ipc::CtrlMessage msg = {};
            msg.type = ipc::CtrlMsgType::LAUNCHER_STOP;
            strncpy(msg.task_name, worker.config.name.c_str(), sizeof(msg.task_name) - 1);
            ipc::send_ctrl_message(worker.ctrl_socket, msg);

            int waited = 0;
            while (waited < 500) {
                int status;
                pid_t result = waitpid(worker.pid, &status, WNOHANG);
                if (result == worker.pid) break;
                usleep(10000);
                waited += 10;
            }

            if (worker.running) {
                kill(worker.pid, SIGKILL);
                waitpid(worker.pid, nullptr, 0);
            }

            close(worker.ctrl_socket);
            worker.running = false;
        }
    }
}

void WorkerManager::check_and_restart() {
    auto now = std::chrono::steady_clock::now();
    for (auto& worker : _workers) {
        if (worker.running) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - worker.last_heartbeat).count();
            if (elapsed > 500) {
                std::cerr << "Worker " << worker.config.name << " heartbeat timeout\n";
                kill(worker.pid, SIGKILL);
                waitpid(worker.pid, nullptr, 0);
                close(worker.ctrl_socket);
                worker.running = false;
            }
        }

        if (!worker.running && worker.config.autorestart
            && worker.restart_count < worker.config.max_restarts) {
            std::cerr << "Restarting worker: " << worker.config.name << "\n";
            start_worker(worker);
        }
    }
}

void WorkerManager::handle_heartbeat(pid_t pid) {
    for (auto& worker : _workers) {
        if (worker.pid == pid) {
            worker.last_heartbeat = std::chrono::steady_clock::now();
            return;
        }
    }
}

void WorkerManager::handle_worker_exit(pid_t pid, int status) {
    for (auto& worker : _workers) {
        if (worker.pid == pid) {
            worker.running = false;
            close(worker.ctrl_socket);

            if (WIFSIGNALED(status)) {
                int sig = WTERMSIG(status);
                std::cerr << "Worker " << worker.config.name
                          << " terminated by signal " << sig << "\n";
            } else if (WIFEXITED(status)) {
                int code = WEXITSTATUS(status);
                std::cerr << "Worker " << worker.config.name
                          << " exited with code " << code << "\n";
            }
            return;
        }
    }
}
