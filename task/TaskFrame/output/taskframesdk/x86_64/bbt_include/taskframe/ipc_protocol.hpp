#pragma once
#include <cstdint>
#include <unistd.h>

namespace ipc {

enum class CtrlMsgType : uint32_t {
    WORKER_REGISTER = 1,
    WORKER_HEARTBEAT,
    WORKER_STATUS,
    LAUNCHER_START,
    LAUNCHER_STOP,
    LAUNCHER_CONFIG,
    LAUNCHER_RESTART,
};

struct CtrlMessage {
    CtrlMsgType type;
    uint32_t    seq;
    pid_t       pid;
    int32_t     status;
    char        task_name[32];
    char        payload[256];
};

ssize_t send_ctrl_message(int fd, const CtrlMessage& msg);
ssize_t recv_ctrl_message(int fd, CtrlMessage& msg, int timeout_ms);

} // namespace ipc
