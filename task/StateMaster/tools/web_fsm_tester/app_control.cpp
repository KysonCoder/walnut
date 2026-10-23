#include "app_control.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "task_frame.hpp"   // SHARED_MEMORY_NAME

namespace web_fsm_tester {

bool process_exists(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return false;
    bool found = false;
    pid_t self = getpid();
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (!isdigit(entry->d_name[0])) continue;
        pid_t pid = atoi(entry->d_name);
        if (pid == self) continue;
        char path[256];
        snprintf(path, sizeof(path), "/proc/%d/cmdline", pid);
        FILE* fp = fopen(path, "r");
        if (!fp) continue;
        char cmdline[256];
        size_t n = fread(cmdline, 1, sizeof(cmdline) - 1, fp);
        fclose(fp);
        if (n == 0) continue;
        cmdline[n] = '\0';
        if (strstr(cmdline, name) != nullptr) { found = true; break; }
    }
    closedir(dir);
    return found;
}

void kill_process(const char* name) {
    DIR* dir = opendir("/proc");
    if (!dir) return;
    pid_t self = getpid();
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (!isdigit(entry->d_name[0]) || (pid_t)atoi(entry->d_name) == self) continue;
        char path[256];
        snprintf(path, sizeof(path), "/proc/%d/cmdline", atoi(entry->d_name));
        FILE* fp = fopen(path, "r");
        if (!fp) continue;
        char cmdline[256];
        size_t n = fread(cmdline, 1, sizeof(cmdline) - 1, fp);
        fclose(fp);
        if (n == 0) continue;
        cmdline[n] = '\0';
        if (strstr(cmdline, name) != nullptr) kill(atoi(entry->d_name), SIGKILL);
    }
    closedir(dir);
}

bool PortInUse(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    bool in_use = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    return in_use;
}

pid_t LauncherStart() {
    shm_unlink(SHARED_MEMORY_NAME);
    kill_process("taskframe_launcher");
    for (const char* elf : kWorkerElfNames) kill_process(elf);
    usleep(200000);

    pid_t pid = fork();
    if (pid == 0) {
        int devnull = open("/dev/null", O_RDONLY);
        if (devnull >= 0) {
            dup2(devnull, STDIN_FILENO);
            close(devnull);
        }
        chdir(kAppDir);
        execl("./taskframe_launcher", "taskframe_launcher", nullptr);
        _exit(127);
    }
    return pid;
}

void LauncherStop(pid_t pid) {
    if (pid > 0) {
        kill(pid, SIGTERM);
        int status = 0;
        int waited = 0;
        while (waited < 15000) {   // 优雅退出最坏 ~10s（每 worker 5s SIGKILL 兜底）
            if (waitpid(pid, &status, WNOHANG) == pid) break;
            usleep(10000);
            waited += 10;
        }
        if (waited >= 15000) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
        }
    }
    for (const char* elf : kWorkerElfNames) kill_process(elf);
    shm_unlink(SHARED_MEMORY_NAME);
}

}  // namespace web_fsm_tester
