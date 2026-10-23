/**
 * @file tf_msg_queue.hpp
 * @brief 跨进程消息队列（新代正式组件）。
 * 底座为 util::RingBufferIpc（SPSC 无锁，跨进程安全）。
 * SPSC 规则：每个任务的收件队列只有主任务可写；注册表写入须持有
 * GlobalData 全局注册互斥锁。
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "utilbase/ring_buffer_ipc.hpp"
#include "utilbase/ring_buffer_ipc_data.hpp"
#include "tf_types.hpp"
#include "status_manager.hpp"

#define TF_MSGQUEUE_REGION_MAGIC 0x54462D4D53470000ULL
#define TF_MSGQUEUE_REGION_VERSION 2

struct Message {
    MsgType type;
    uint8_t priority;
    uint16_t payload_size;
    uint32_t seqnum;
    uint64_t timestamp_ns;
    alignas(8) char payload[512];
    Message() : type(MsgType::EMPTY), priority(0), payload_size(0), seqnum(0), timestamp_ns(0) {
        memset(payload, 0, sizeof(payload));
    }
};
static_assert(std::is_trivially_copyable_v<Message>, "Message must be shm-safe");

struct TaskMsgQueue {
    util::RingBufferIpcData<Message, 32> ring;
};

struct MsgQueueRegion {
    uint64_t magic;
    uint32_t version;
    uint32_t max_tasks;
    uint32_t _pad[2];
    TaskNameRegistry registry;
};

class TfMsgQueueManager {
public:
    TfMsgQueueManager();
    bool Init(void* region_base, size_t region_size, int max_tasks);  // 写端
    bool Attach(void* region_base, size_t region_size);               // 读端
    bool IsWriter() const { return _is_writer; }

    bool RegisterQueue(int task_id, const char* name);
    int FindQueue(const char* name) const;

    bool Send(int task_id, MsgType type, const void* payload = nullptr,
              size_t len = 0, uint8_t priority = 0);
    bool Send(const char* task_name, MsgType type, const void* payload = nullptr,
              size_t len = 0, uint8_t priority = 0);
    bool Receive(int task_id, Message& out);
    bool HasMessage(int task_id) const;

private:
    MsgQueueRegion* _header;
    TaskMsgQueue* _queues;
    int _max_tasks;
    uint32_t _next_seqnum;
    bool _is_writer;
    uint64_t CurrentTimeNs() const;
};
