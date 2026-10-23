#include <gtest/gtest.h>
#include "ipc_protocol.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>

TEST(IpcProtocol, SendRecv) {
    int sv[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    ipc::CtrlMessage send_msg = {};
    send_msg.type = ipc::CtrlMsgType::WORKER_REGISTER;
    send_msg.pid = 12345;
    send_msg.seq = 1;
    strcpy(send_msg.task_name, "test_task");

    ssize_t sent = ipc::send_ctrl_message(sv[0], send_msg);
    EXPECT_EQ(sent, sizeof(send_msg));

    ipc::CtrlMessage recv_msg = {};
    ssize_t recvd = ipc::recv_ctrl_message(sv[1], recv_msg, 1000);
    EXPECT_EQ(recvd, sizeof(recv_msg));
    EXPECT_EQ(recv_msg.type, ipc::CtrlMsgType::WORKER_REGISTER);
    EXPECT_EQ(recv_msg.pid, 12345);
    EXPECT_EQ(recv_msg.seq, 1);
    EXPECT_STREQ(recv_msg.task_name, "test_task");

    close(sv[0]);
    close(sv[1]);
}

TEST(IpcProtocol, Timeout) {
    int sv[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    ipc::CtrlMessage msg = {};
    auto start = std::chrono::steady_clock::now();
    ssize_t recvd = ipc::recv_ctrl_message(sv[1], msg, 50);
    auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(recvd, 0);  // timeout
    EXPECT_LT(elapsed, std::chrono::milliseconds(200));

    close(sv[0]);
    close(sv[1]);
}
