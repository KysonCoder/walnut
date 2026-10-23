#include "ipc_protocol.hpp"
#include <sys/socket.h>
#include <poll.h>
#include <cstring>

namespace ipc {

ssize_t send_ctrl_message(int fd, const CtrlMessage& msg) {
    return send(fd, &msg, sizeof(msg), 0);
}

ssize_t recv_ctrl_message(int fd, CtrlMessage& msg, int timeout_ms) {
    if (timeout_ms >= 0) {
        struct pollfd pfd = {fd, POLLIN, 0};
        int ret = poll(&pfd, 1, timeout_ms);
        if (ret <= 0) return ret;  // timeout or error
    }
    return recv(fd, &msg, sizeof(msg), 0);
}

} // namespace ipc
