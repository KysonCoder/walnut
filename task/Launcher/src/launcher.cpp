#include "launcher.hpp"
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>

#include "tf_event.hpp"
#include "tf_shm_region.hpp"
#include "shm_header.hpp"
#include "shm_ptr.hpp"
#include "task_frame.hpp"      // GlobalData
#include "status_manager.hpp"  // StatusManagerData / TASK_STATUS_TYPE

namespace {

// HMI 是状态切换的唯一官方命令通道（SPSC 规则：只有主任务可写事件环，
// Launcher 经它转发而不是自己 PublishBroadcast）。成功收到 HTTP 200 返回 true。
bool hmi_send_state_msg(const char* state_msg) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8080);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return false;
    }
    char body[128];
    int body_len = snprintf(body, sizeof(body),
                            R"({"cmd":"REFSTATECHANGE","stateMSG":"%s"})", state_msg);
    char req[512];
    int req_len = snprintf(req, sizeof(req),
                           "POST /stateMsg HTTP/1.1\r\nHost: 127.0.0.1\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\nConnection: close\r\n\r\n%s",
                           body_len, body);
    bool ok = write(fd, req, static_cast<size_t>(req_len)) == req_len;
    if (ok) {
        pollfd pfd{fd, POLLIN, 0};
        if (poll(&pfd, 1, 2000) > 0) {
            char resp[256] = {};
            ssize_t n = read(fd, resp, sizeof(resp) - 1);
            ok = n > 0 && strncmp(resp, "HTTP/1.1 200", 12) == 0;
        } else {
            ok = false;
        }
    }
    close(fd);
    return ok;
}

}  // namespace

bool Launcher::init(const std::string& config_path) {
    if (!parse_config(config_path, _config)) {
        return false;
    }
    return true;
}

bool Launcher::init_with_default() {
    load_default_config(_config);
    return true;
}

bool Launcher::attach_shared_memory() {
    if (_shm_base != nullptr && _shm_base != MAP_FAILED) {
        return true;
    }

    int retries = 100;
    int shm_fd = -1;
    while (retries > 0) {
        shm_fd = shm_open(_config.system.shm_name.c_str(), O_RDWR, 0666);
        if (shm_fd >= 0) {
            break;
        }
        usleep(100000);
        retries--;
    }

    if (shm_fd < 0) {
        std::cerr << "Failed to attach shared memory after 10s: "
                  << _config.system.shm_name << "\n";
        return false;
    }

    struct stat st;
    if (fstat(shm_fd, &st) < 0) {
        perror("fstat");
        close(shm_fd);
        return false;
    }
    size_t shm_size = static_cast<size_t>(st.st_size);

    void* shm_base = mmap(nullptr, shm_size,
                          PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    close(shm_fd);

    if (shm_base == MAP_FAILED) {
        perror("mmap");
        return false;
    }

    // Validate header magic
    ShmHeader* header = static_cast<ShmHeader*>(shm_base);
    if (header->magic != SHM_MAGIC) {
        std::cerr << "Shared memory header magic mismatch\n";
        munmap(shm_base, shm_size);
        return false;
    }

    _shm_base = shm_base;
    g_shm_base = shm_base;
    g_shm_size = shm_size;

    return true;
}

bool Launcher::try_attach_shared_memory_once() {
    if (_shm_base != nullptr && _shm_base != MAP_FAILED) return true;
    int fd = shm_open(_config.system.shm_name.c_str(), O_RDONLY, 0);
    if (fd < 0) return false;
    struct stat st;
    if (fstat(fd, &st) < 0) { close(fd); return false; }
    void* base = mmap(nullptr, static_cast<size_t>(st.st_size),
                      PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (base == MAP_FAILED) return false;
    ShmHeader* header = static_cast<ShmHeader*>(base);
    if (header->magic != SHM_MAGIC) {
        munmap(base, static_cast<size_t>(st.st_size));
        return false;
    }
    _shm_base = base;
    g_shm_base = base;
    g_shm_size = static_cast<size_t>(st.st_size);
    return true;
}

int Launcher::read_all_task_status(int* out, int max_count) {
    if (_shm_base == nullptr || _shm_base == MAP_FAILED) return -1;
    GlobalData* gd = reinterpret_cast<GlobalData*>(
        static_cast<char*>(_shm_base) + sizeof(ShmHeader));
    size_t off = gd->_task_status.offset();
    if (off == 0) return -1;
    StatusManagerData* smd = reinterpret_cast<StatusManagerData*>(
        static_cast<char*>(_shm_base) + off);
    if (smd->magic != STATUS_MANAGER_MAGIC) return -1;
    int count = smd->task_number;
    if (count > MAX_TASK_NUMBER || count > max_count) count =
        max_count < MAX_TASK_NUMBER ? max_count : MAX_TASK_NUMBER;
    for (int i = 0; i < count; i++) {
        out[i] = smd->task_status[i]._status_value.load();
    }
    return count;
}

bool Launcher::any_task_ready_or_running() {
    int status[MAX_TASK_NUMBER];
    int count = read_all_task_status(status, MAX_TASK_NUMBER);
    for (int i = 0; i < count; i++) {
        if (status[i] == READY || status[i] == RUNNING) return true;
    }
    return false;
}

bool Launcher::all_tasks_motor_off_or_error() {
    int status[MAX_TASK_NUMBER];
    int count = read_all_task_status(status, MAX_TASK_NUMBER);
    if (count <= 0) return false;
    for (int i = 0; i < count; i++) {
        if (status[i] != MOTOR_OFF && status[i] != ERROR) return false;
    }
    return true;
}

void Launcher::graceful_motor_off() {
    // shm 不存在（应用根本没起来过）→ 没有可下电的轴，直接返回
    if (!try_attach_shared_memory_once()) return;
    // 没有 READY/RUNNING 的任务 → 本来就处于下电状态，无需等待
    if (!any_task_ready_or_running()) return;

    if (!hmi_send_state_msg("msg_motor_off")) {
        std::cerr << "graceful_motor_off: HMI unreachable, "
                     "skipping graceful motor-off\n";
        return;
    }
    // 等全部任务落到 MOTOR_OFF（ERROR 也算停稳），最多 3s，超时直接继续
    for (int waited = 0; waited < 3000; waited += 50) {
        if (all_tasks_motor_off_or_error()) {
            std::cout << "All tasks reached MOTOR_OFF before shutdown\n";
            return;
        }
        usleep(50000);
    }
    std::cerr << "graceful_motor_off: timeout waiting for MOTOR_OFF, "
                 "stopping anyway\n";
}

bool Launcher::CleanupStaleShm(const std::string& shm_name) {
    int fd = shm_open(shm_name.c_str(), O_RDWR, 0);
    if (fd < 0 && errno == ENOENT) {
        return true;  // 无残留
    }
    if (fd >= 0) {
        close(fd);
    }
    // fd < 0 且 errno == EACCES：典型的 root 残留（0644 root 拥有）。
    // /dev/shm 为 sticky 目录，仅 owner/root 可 unlink——当前用户是 owner
    // 时即使无读写权限也能删掉；不是 owner 时 unlink 失败，只能提示手工清理。
    if (shm_unlink(shm_name.c_str()) == 0) {
        std::cout << "Removed stale shared memory: " << shm_name << "\n";
        return true;
    }
    std::string path = shm_name[0] == '/' ? shm_name.substr(1) : shm_name;
    std::cerr << "Stale shared memory " << shm_name
              << " cannot be removed (owned by another user?). "
              << "Please run: sudo rm /dev/shm/" << path << "\n";
    return false;
}

bool Launcher::start_all_workers() {
    if (!CleanupStaleShm(_config.system.shm_name)) {
        return false;
    }
    return _worker_mgr.start_all(_config);
}

void Launcher::stop_all_workers() {
    graceful_motor_off();

    _worker_mgr.stop_all();
    if (_shm_base && _shm_base != MAP_FAILED) {
        munmap(_shm_base, g_shm_size);
    }
    shm_unlink(_config.system.shm_name.c_str());
}

void Launcher::reap_workers() {
    int status;
    pid_t pid;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        _worker_mgr.handle_worker_exit(pid, status);
    }
}

void Launcher::run_once() {
    // Reap any exited children
    reap_workers();

    // If any worker has crashed, forward ERROR via HMI
    // （state_master 活着时由它正规广播；HMI 不可达说明主任务也已死，仅记日志）。
    // 每个崩溃 episode 只通知一次，避免 100ms 循环刷屏。
    bool any_crashed = false;
    for (const auto& worker : _worker_mgr.workers()) {
        if (!worker.running && worker.restart_count > 0) {
            any_crashed = true;
            break;
        }
    }
    if (any_crashed && !_crash_error_notified) {
        if (hmi_send_state_msg("msg_error")) {
            std::cerr << "Worker crashed, ERROR forwarded via HMI\n";
        } else {
            std::cerr << "Worker crashed, HMI unreachable for ERROR notification\n";
        }
        _crash_error_notified = true;
    } else if (!any_crashed) {
        _crash_error_notified = false;
    }

    // Check heartbeats and restart
    _worker_mgr.check_and_restart();

    // Poll for control messages from workers
    for (auto& worker : _worker_mgr.workers()) {
        if (!worker.running || worker.ctrl_socket < 0) continue;

        ipc::CtrlMessage msg;
        ssize_t ret = ipc::recv_ctrl_message(worker.ctrl_socket, msg, 0);
        if (ret == sizeof(msg)) {
            switch (msg.type) {
                case ipc::CtrlMsgType::WORKER_REGISTER:
                    std::cout << "Worker registered: " << msg.task_name
                              << " (pid=" << msg.pid << ")\n";
                    break;
                case ipc::CtrlMsgType::WORKER_HEARTBEAT:
                    _worker_mgr.handle_heartbeat(msg.pid);
                    break;
                default:
                    break;
            }
        }
    }
}
