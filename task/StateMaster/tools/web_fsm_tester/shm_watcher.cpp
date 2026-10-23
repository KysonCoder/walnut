#include "shm_watcher.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace web_fsm_tester {

bool ShmStatusWatcher::Attach() {
    if (base_ != nullptr) return true;
    int fd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0);
    if (fd < 0) return false;
    ShmHeader header{};
    ssize_t n = pread(fd, &header, sizeof(header), 0);
    if (n != (ssize_t)sizeof(header) || header.magic != SHM_MAGIC || header.size == 0) {
        close(fd);
        return false;
    }
    size_ = header.size;
    void* base = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (base == MAP_FAILED) return false;
    base_ = base;
    global_ = reinterpret_cast<GlobalData*>(static_cast<char*>(base_) + sizeof(ShmHeader));
    return true;
}

void ShmStatusWatcher::Detach() {
    if (base_ != nullptr) { munmap(base_, size_); base_ = nullptr; }
}

StatusManagerData* ShmStatusWatcher::StatusData() {
    if (base_ == nullptr) return nullptr;
    size_t off = global_->_task_status.offset();
    if (off == 0) return nullptr;
    return reinterpret_cast<StatusManagerData*>(static_cast<char*>(base_) + off);
}

bool ShmStatusWatcher::StatusTableReady() {
    if (!Attach()) return false;
    StatusManagerData* smd = StatusData();
    if (smd == nullptr || smd->magic != STATUS_MANAGER_MAGIC) return false;
    for (const char* name : kTaskNames) {
        int dummy;
        if (!GetStatusByName(name, &dummy)) return false;
    }
    return true;
}

bool ShmStatusWatcher::GetStatusByName(const char* name, int* out) {
    StatusManagerData* smd = StatusData();
    if (smd == nullptr) return false;
    int count = smd->task_number;
    if (count > MAX_TASK_NUMBER) count = MAX_TASK_NUMBER;
    for (int i = 0; i < count; i++) {
        if (strncmp(name, smd->task_status[i]._task_name, 32) == 0) {
            *out = smd->task_status[i]._status_value.load();
            return true;
        }
    }
    return false;
}

std::vector<TaskStatusInfo> ShmStatusWatcher::ReadAll() {
    std::vector<TaskStatusInfo> out;
    for (const char* name : kTaskNames) {
        int v = -1;
        GetStatusByName(name, &v);
        out.push_back({name, v});
    }
    return out;
}

bool ShmStatusWatcher::WaitForAllTasks(int expected, int timeout_ms,
                                       const std::function<bool()>& abort) {
    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (abort && abort()) return false;
        bool all = true;
        for (const char* name : kTaskNames) {
            int v = -1;
            if (!GetStatusByName(name, &v) || v != expected) { all = false; break; }
        }
        if (all) return true;
        usleep(20000);
    }
    return false;
}

std::string ShmStatusWatcher::Snapshot() {
    std::string s;
    char line[128];
    for (const char* name : kTaskNames) {
        int v = -1;
        bool ok = GetStatusByName(name, &v);
        snprintf(line, sizeof(line), "%s=%d ", name, ok ? v : -1);
        s += line;
    }
    return s;
}

}  // namespace web_fsm_tester
