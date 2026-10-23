// 最小心跳 worker：注册后每 ~50ms 发心跳，收到 LAUNCHER_STOP 正常退出。
// 供 worker_manager_test 驱动 WorkerManager 的正常生命周期路径。
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "ipc_protocol.hpp"

int main(int argc, char* argv[]) {
    int ctrl_fd = 3;
    const char* name = "test_worker";
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--ctrl-fd=", 10) == 0) ctrl_fd = atoi(argv[i] + 10);
        if (strncmp(argv[i], "--name=", 7) == 0) name = argv[i] + 7;
    }

    ipc::CtrlMessage msg = {};
    msg.type = ipc::CtrlMsgType::WORKER_REGISTER;
    msg.pid = getpid();
    strncpy(msg.task_name, name, sizeof(msg.task_name) - 1);
    ipc::send_ctrl_message(ctrl_fd, msg);

    for (;;) {
        ipc::CtrlMessage hb = {};
        hb.type = ipc::CtrlMsgType::WORKER_HEARTBEAT;
        hb.pid = getpid();
        strncpy(hb.task_name, name, sizeof(hb.task_name) - 1);
        ipc::send_ctrl_message(ctrl_fd, hb);

        ipc::CtrlMessage in;
        if (ipc::recv_ctrl_message(ctrl_fd, in, 50) ==
                static_cast<ssize_t>(sizeof(in)) &&
            in.type == ipc::CtrlMsgType::LAUNCHER_STOP) {
            return 0;
        }
    }
}
