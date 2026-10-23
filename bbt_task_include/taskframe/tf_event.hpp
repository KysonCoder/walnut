/**
 * @file tf_event.hpp
 * @brief 跨进程事件管理器（新代正式组件）。
 * 底座为 util::RingBufferIpc（SPSC 无锁，跨进程安全）。
 * SPSC 规则：每个任务的收件环只有主任务（Init 调用方）可写；
 * 任务间事件须经主任务转发。注册表写入须持有 GlobalData 全局注册互斥锁。
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <type_traits>
#include <unistd.h>

#include "utilbase/ring_buffer_ipc.hpp"
#include "utilbase/ring_buffer_ipc_data.hpp"
#include "tf_types.hpp"
#include "status_manager.hpp"

#define TF_EVENT_REGION_MAGIC 0x54462D4556540000ULL
#define TF_EVENT_REGION_VERSION 2

constexpr uint32_t EVENT_FLAG_REQUIRE_ACK = 1u << 0;
constexpr uint32_t EVENT_FLAG_BROADCAST   = 1u << 1;

enum class EventSyncMode : uint8_t {
    FIRE_AND_FORGET = 0, STATUS_MIRROR = 1, EXPLICIT_ACK = 2,
};

struct Event {
    EventType type;
    uint32_t seqnum;
    uint64_t timestamp_ns;
    uint32_t flags;
    char payload[64];
    Event() : type(EventType::EMPTY), seqnum(0), timestamp_ns(0), flags(0) {
        memset(payload, 0, sizeof(payload));
    }
};
static_assert(std::is_trivially_copyable_v<Event>, "Event must be shm-safe");

struct TaskEventRing {
    util::RingBufferIpcData<Event, 16> ring;
};

struct EventAckSlot {
    alignas(64) std::atomic<int32_t> last_ack_seq{-1};
    alignas(64) std::atomic<uint32_t> pending_count{0};
};

struct EventRegion {
    uint64_t magic;
    uint32_t version;
    uint32_t max_tasks;
    uint32_t sync_mode;
    uint32_t _pad;
    TaskNameRegistry registry;
};

class TfEventManager {
public:
    TfEventManager();
    // 写端（主任务）初始化：建 region、清零注册表、初始化所有环
    bool Init(void* region_base, size_t region_size, int max_tasks);
    // 读端（子任务）附加：校验 magic/version
    bool Attach(void* region_base, size_t region_size);
    bool IsWriter() const { return _is_writer; }

    // 注册表：task_id 即环索引，保证 ConsumeEvent(task_id) 与按名发布一致。
    // 调用方必须持有 GlobalData 全局注册互斥锁（跨进程串行化）。
    bool RegisterTask(int task_id, const char* name);
    int FindTask(const char* name) const;

    bool PublishEvent(int task_id, EventType type, const void* payload = nullptr,
                      size_t payload_len = 0, uint32_t flags = 0);
    bool PublishEvent(const char* task_name, EventType type,
                      const void* payload = nullptr, size_t payload_len = 0,
                      uint32_t flags = 0);
    // 仅写端可调用；对任一任务推送失败则整体返回 false 并逐任务 Log_E
    bool PublishBroadcast(EventType type, const void* payload = nullptr,
                          size_t payload_len = 0, uint32_t flags = 0);

    bool ConsumeEvent(int task_id, Event& out);
    bool HasEvent(int task_id) const;

    void SetSyncMode(EventSyncMode mode);
    EventSyncMode GetSyncMode() const;
    bool WaitAck(int task_id, uint32_t seqnum, uint32_t timeout_ms);
    void SendAck(int task_id, uint32_t seqnum);

private:
    EventRegion* _header;
    TaskEventRing* _rings;
    EventAckSlot* _ack_slots;
    int _max_tasks;
    uint32_t _next_seqnum;
    bool _is_writer;
    uint64_t CurrentTimeNs() const;
};
