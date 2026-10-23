// 沉默 worker：注册后不再发任何消息，用于触发 WorkerManager 的心跳超时路径。
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "ipc_protocol.hpp"

int main(int argc, char* argv[]) {
    int ctrl_fd = 3;
    const char* name = "silent_worker";
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) ctrl_fd = atoi(argv[i] + 10);
        if (strncmp(argv[i], "--name=", 7) == 0) name = argv[i] + 7;
    }

    ipc::CtrlMessage msg = {};
    msg.type = ipc::CtrlMsgType::WORKER_REGISTER;
    msg.pid = getpid();
    strncpy(msg.task_name, name, sizeof(msg.task_name) - 1);
    ipc::send_ctrl_message(ctrl_fd, msg);

    for (;;) pause();
}
