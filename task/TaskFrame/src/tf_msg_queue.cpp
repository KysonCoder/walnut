#include "tf_msg_queue.hpp"
#include <ctime>
#include <new>
#include "utilbase/logger.h"

TfMsgQueueManager::TfMsgQueueManager()
    : _header(nullptr), _queues(nullptr), _max_tasks(0),
      _next_seqnum(1), _is_writer(false) {
}

bool TfMsgQueueManager::Init(void* region_base, size_t region_size, int max_tasks) {
    if (!region_base || region_size == 0 || max_tasks <= 0 || max_tasks > MAX_TASK_NUMBER) {
        return false;
    }

    size_t header_size = sizeof(MsgQueueRegion);
    size_t queues_size = static_cast<size_t>(max_tasks) * sizeof(TaskMsgQueue);
    if (region_size < header_size + queues_size) {
        Log_E("TfMsgQueueManager::Init: region too small (%zu < %zu)\n",
              region_size, header_size + queues_size);
        return false;
    }

    _header = static_cast<MsgQueueRegion*>(region_base);
    _header->magic = TF_MSGQUEUE_REGION_MAGIC;
    _header->version = TF_MSGQUEUE_REGION_VERSION;
    _header->max_tasks = static_cast<uint32_t>(max_tasks);
    _header->_pad[0] = 0;
    _header->_pad[1] = 0;
    new (&_header->registry) TaskNameRegistry();

    _queues = reinterpret_cast<TaskMsgQueue*>(
        static_cast<uint8_t*>(region_base) + header_size);

    for (int i = 0; i < max_tasks; ++i) {
        new (&_queues[i]) TaskMsgQueue();
        util::RingBufferIpc<Message, 32> ring(&_queues[i].ring);
        ring.Init();  // 写端初始化读写索引
    }

    _max_tasks = max_tasks;
    _next_seqnum = 1;
    _is_writer = true;
    return true;
}

bool TfMsgQueueManager::Attach(void* region_base, size_t region_size) {
    if (!region_base || region_size == 0) {
        return false;
    }

    MsgQueueRegion* hdr = static_cast<MsgQueueRegion*>(region_base);
    if (hdr->magic != TF_MSGQUEUE_REGION_MAGIC) {
        Log_E("TfMsgQueueManager::Attach: bad magic\n");
        return false;
    }
    if (hdr->version != TF_MSGQUEUE_REGION_VERSION) {
        Log_E("TfMsgQueueManager::Attach: version mismatch (%u != %u)\n",
              hdr->version, TF_MSGQUEUE_REGION_VERSION);
        return false;
    }

    int max_tasks = static_cast<int>(hdr->max_tasks);
    if (max_tasks <= 0 || max_tasks > MAX_TASK_NUMBER) {
        return false;
    }

    size_t header_size = sizeof(MsgQueueRegion);
    size_t queues_size = static_cast<size_t>(max_tasks) * sizeof(TaskMsgQueue);
    if (region_size < header_size + queues_size) {
        return false;
    }

    _header = hdr;
    _queues = reinterpret_cast<TaskMsgQueue*>(
        static_cast<uint8_t*>(region_base) + header_size);
    _max_tasks = max_tasks;
    _next_seqnum = 1;
    _is_writer = false;

    for (int i = 0; i < _max_tasks; ++i) {
        util::RingBufferIpc<Message, 32> ring(&_queues[i].ring);
        ring.Attach();
    }
    return true;
}

bool TfMsgQueueManager::RegisterQueue(int task_id, const char* name) {
    if (!_header || !name || name[0] == '\0' || task_id < 0 || task_id >= _max_tasks) {
        return false;
    }
    int existing = FindQueue(name);
    if (existing >= 0) {
        if (existing != task_id) {
            Log_W("TfMsgQueueManager::RegisterQueue: name '%s' already at id %d\n", name, existing);
            return false;
        }
        return true;  // 幂等
    }
    // 调用方持有全局注册互斥锁，此处无需额外同步
    TaskNameSlot& slot = _header->registry.slots[task_id];
    std::strncpy(slot.name, name, sizeof(slot.name) - 1);
    slot.name[sizeof(slot.name) - 1] = '\0';
    slot.ready.store(1, std::memory_order_release);
    uint32_t need = static_cast<uint32_t>(task_id) + 1;
    uint32_t cur = _header->registry.count.load(std::memory_order_relaxed);
    if (need > cur) {
        _header->registry.count.store(need, std::memory_order_relaxed);
    }
    return true;
}

int TfMsgQueueManager::FindQueue(const char* name) const {
    if (!_header || !name) {
        return -1;
    }
    uint32_t n = _header->registry.count.load(std::memory_order_acquire);
    for (uint32_t i = 0; i < n && i < static_cast<uint32_t>(_max_tasks); ++i) {
        if (_header->registry.slots[i].ready.load(std::memory_order_acquire) == 1 &&
            std::strncmp(_header->registry.slots[i].name, name, 32) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool TfMsgQueueManager::Send(int task_id, MsgType type, const void* payload,
                             size_t len, uint8_t priority) {
    if (task_id < 0 || task_id >= _max_tasks || !_queues) {
        return false;
    }
    if (len > sizeof(Message::payload)) {
        return false;
    }

    Message msg;
    msg.type = type;
    msg.priority = priority;
    msg.payload_size = static_cast<uint16_t>(len);
    msg.seqnum = _next_seqnum++;
    msg.timestamp_ns = CurrentTimeNs();
    if (payload && len > 0) {
        memcpy(msg.payload, payload, len);
    }

    util::RingBufferIpc<Message, 32> ring(&_queues[task_id].ring);
    return ring.TryPush(msg);
}

bool TfMsgQueueManager::Send(const char* task_name, MsgType type, const void* payload,
                             size_t len, uint8_t priority) {
    int task_id = FindQueue(task_name);
    if (task_id < 0) {
        Log_E("TfMsgQueueManager::Send: task '%s' not registered\n", task_name);
        return false;
    }
    return Send(task_id, type, payload, len, priority);
}

bool TfMsgQueueManager::Receive(int task_id, Message& out) {
    if (task_id < 0 || task_id >= _max_tasks || !_queues) {
        return false;
    }
    util::RingBufferIpc<Message, 32> ring(&_queues[task_id].ring);
    return ring.TryPop(out);
}

bool TfMsgQueueManager::HasMessage(int task_id) const {
    if (task_id < 0 || task_id >= _max_tasks || !_queues) {
        return false;
    }
    util::RingBufferIpc<Message, 32> ring(
        const_cast<util::RingBufferIpcData<Message, 32>*>(&_queues[task_id].ring));
    return !ring.IsEmpty();
}

uint64_t TfMsgQueueManager::CurrentTimeNs() const {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}
