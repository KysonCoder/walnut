#include "tf_event.hpp"
#include <new>
#include <cstring>
#include "utilbase/logger.h"

TfEventManager::TfEventManager()
    : _header(nullptr), _rings(nullptr), _ack_slots(nullptr),
      _max_tasks(0), _next_seqnum(1), _is_writer(false) {
}

bool TfEventManager::Init(void* region_base, size_t region_size, int max_tasks) {
    if (!region_base || region_size == 0 || max_tasks <= 0 || max_tasks > MAX_TASK_NUMBER) {
        return false;
    }

    size_t required = sizeof(EventRegion)
                    + sizeof(TaskEventRing) * max_tasks
                    + sizeof(EventAckSlot) * max_tasks;
    if (region_size < required) {
        Log_E("TfEventManager::Init: region too small (%zu < %zu)\n", region_size, required);
        return false;
    }

    _header = static_cast<EventRegion*>(region_base);
    _header->magic = TF_EVENT_REGION_MAGIC;
    _header->version = TF_EVENT_REGION_VERSION;
    _header->max_tasks = static_cast<uint32_t>(max_tasks);
    _header->sync_mode = static_cast<uint32_t>(EventSyncMode::FIRE_AND_FORGET);
    _header->_pad = 0;
    new (&_header->registry) TaskNameRegistry();

    uint8_t* base = static_cast<uint8_t*>(region_base);
    _rings = reinterpret_cast<TaskEventRing*>(base + sizeof(EventRegion));
    _ack_slots = reinterpret_cast<EventAckSlot*>(
        base + sizeof(EventRegion) + sizeof(TaskEventRing) * max_tasks);
    _max_tasks = max_tasks;
    _next_seqnum = 1;
    _is_writer = true;

    for (int i = 0; i < max_tasks; ++i) {
        new (&_rings[i]) TaskEventRing();
        util::RingBufferIpc<Event, 16> ring(&_rings[i].ring);
        ring.Init();  // 写端初始化读写索引
        new (&_ack_slots[i]) EventAckSlot();
    }
    return true;
}

bool TfEventManager::Attach(void* region_base, size_t region_size) {
    if (!region_base || region_size == 0) {
        return false;
    }

    EventRegion* hdr = static_cast<EventRegion*>(region_base);
    if (hdr->magic != TF_EVENT_REGION_MAGIC) {
        Log_E("TfEventManager::Attach: bad magic\n");
        return false;
    }
    if (hdr->version != TF_EVENT_REGION_VERSION) {
        Log_E("TfEventManager::Attach: version mismatch (%u != %u)\n",
              hdr->version, TF_EVENT_REGION_VERSION);
        return false;
    }
    if (hdr->max_tasks == 0 || hdr->max_tasks > MAX_TASK_NUMBER) {
        return false;
    }

    size_t required = sizeof(EventRegion)
                    + sizeof(TaskEventRing) * hdr->max_tasks
                    + sizeof(EventAckSlot) * hdr->max_tasks;
    if (region_size < required) {
        return false;
    }

    _header = hdr;
    uint8_t* base = static_cast<uint8_t*>(region_base);
    _rings = reinterpret_cast<TaskEventRing*>(base + sizeof(EventRegion));
    _ack_slots = reinterpret_cast<EventAckSlot*>(
        base + sizeof(EventRegion) + sizeof(TaskEventRing) * hdr->max_tasks);
    _max_tasks = static_cast<int>(hdr->max_tasks);
    _next_seqnum = 1;
    _is_writer = false;

    for (int i = 0; i < _max_tasks; ++i) {
        util::RingBufferIpc<Event, 16> ring(&_rings[i].ring);
        ring.Attach();
    }
    return true;
}

bool TfEventManager::RegisterTask(int task_id, const char* name) {
    if (!_header || !name || name[0] == '\0' || task_id < 0 || task_id >= _max_tasks) {
        return false;
    }
    int existing = FindTask(name);
    if (existing >= 0) {
        if (existing != task_id) {
            Log_W("TfEventManager::RegisterTask: name '%s' already at id %d\n", name, existing);
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

int TfEventManager::FindTask(const char* name) const {
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

bool TfEventManager::PublishEvent(int task_id, EventType type, const void* payload,
                                  size_t payload_len, uint32_t flags) {
    if (task_id < 0 || task_id >= _max_tasks || !_rings) {
        return false;
    }

    Event ev;
    ev.type = type;
    ev.seqnum = _next_seqnum++;
    ev.timestamp_ns = CurrentTimeNs();
    ev.flags = flags;
    if (payload && payload_len > 0) {
        size_t copy_len = payload_len < sizeof(ev.payload) ? payload_len : sizeof(ev.payload);
        std::memcpy(ev.payload, payload, copy_len);
    }

    util::RingBufferIpc<Event, 16> ring(&_rings[task_id].ring);
    return ring.TryPush(ev);
}

bool TfEventManager::PublishEvent(const char* task_name, EventType type,
                                  const void* payload, size_t payload_len, uint32_t flags) {
    int task_id = FindTask(task_name);
    if (task_id < 0) {
        Log_E("TfEventManager::PublishEvent: task '%s' not registered\n", task_name);
        return false;
    }
    return PublishEvent(task_id, type, payload, payload_len, flags);
}

bool TfEventManager::PublishBroadcast(EventType type, const void* payload,
                                      size_t payload_len, uint32_t flags) {
    if (!_is_writer) {
        Log_E("TfEventManager::PublishBroadcast: only the writer (main task) may broadcast\n");
        return false;
    }
    bool all_ok = true;
    for (int i = 0; i < _max_tasks; ++i) {
        if (!PublishEvent(i, type, payload, payload_len, flags | EVENT_FLAG_BROADCAST)) {
            Log_E("TfEventManager::PublishBroadcast: ring full for task %d\n", i);
            all_ok = false;
        }
    }
    return all_ok;
}

bool TfEventManager::ConsumeEvent(int task_id, Event& out) {
    if (task_id < 0 || task_id >= _max_tasks || !_rings) {
        return false;
    }
    util::RingBufferIpc<Event, 16> ring(&_rings[task_id].ring);
    return ring.TryPop(out);
}

bool TfEventManager::HasEvent(int task_id) const {
    if (task_id < 0 || task_id >= _max_tasks || !_rings) {
        return false;
    }
    util::RingBufferIpc<Event, 16> ring(
        const_cast<util::RingBufferIpcData<Event, 16>*>(&_rings[task_id].ring));
    return !ring.IsEmpty();
}

void TfEventManager::SetSyncMode(EventSyncMode mode) {
    if (_header) {
        _header->sync_mode = static_cast<uint32_t>(mode);
    }
}

EventSyncMode TfEventManager::GetSyncMode() const {
    if (!_header) {
        return EventSyncMode::FIRE_AND_FORGET;
    }
    return static_cast<EventSyncMode>(_header->sync_mode);
}

bool TfEventManager::WaitAck(int task_id, uint32_t seqnum, uint32_t timeout_ms) {
    if (task_id < 0 || task_id >= _max_tasks || !_ack_slots) {
        return false;
    }

    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        if (static_cast<uint32_t>(_ack_slots[task_id].last_ack_seq.load()) >= seqnum) {
            return true;
        }
        usleep(1000);
        ++elapsed;
    }
    return false;
}

void TfEventManager::SendAck(int task_id, uint32_t seqnum) {
    if (task_id < 0 || task_id >= _max_tasks || !_ack_slots) {
        return;
    }
    _ack_slots[task_id].last_ack_seq.store(static_cast<int32_t>(seqnum));
}

uint64_t TfEventManager::CurrentTimeNs() const {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}
