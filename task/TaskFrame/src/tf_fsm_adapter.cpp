#include "tf_fsm_adapter.hpp"
#include "task_frame.hpp"

TfFsmEventAdapter::TfFsmEventAdapter() {
    for (int i = 0; i < MAX_EVENT_TYPE; ++i) {
        _handlers[i].action = nullptr;
        _handlers[i].expected_state = TASK_STATUS_TYPE::UNDEFINED;
        _handlers[i].registered = false;
    }
}

void TfFsmEventAdapter::RegisterEventHandler(EventType event, ActionFunc action, TASK_STATUS_TYPE expected_state) {
    int idx = static_cast<int>(event);
    if (idx >= 0 && idx < MAX_EVENT_TYPE) {
        _handlers[idx].action = action;
        _handlers[idx].expected_state = expected_state;
        _handlers[idx].registered = true;
    }
}

void TfFsmEventAdapter::UnregisterEventHandler(EventType event) {
    int idx = static_cast<int>(event);
    if (idx >= 0 && idx < MAX_EVENT_TYPE) {
        _handlers[idx].registered = false;
        _handlers[idx].action = nullptr;
        _handlers[idx].expected_state = TASK_STATUS_TYPE::UNDEFINED;
    }
}

void TfFsmEventAdapter::OnEventReceived(const Event& event, CTaskFrame* task) {
    int idx = static_cast<int>(event.type);
    if (idx >= 0 && idx < MAX_EVENT_TYPE) {
        const HandlerEntry& entry = _handlers[idx];
        if (entry.registered && entry.action) {
            entry.action(task);
        }
    }
}

TASK_STATUS_TYPE TfFsmEventAdapter::GetExpectedState(EventType event) const {
    int idx = static_cast<int>(event);
    if (idx >= 0 && idx < MAX_EVENT_TYPE) {
        const HandlerEntry& entry = _handlers[idx];
        if (entry.registered) {
            return entry.expected_state;
        }
    }
    return TASK_STATUS_TYPE::UNDEFINED;
}

bool TfFsmEventAdapter::HasHandler(EventType event) const {
    int idx = static_cast<int>(event);
    if (idx >= 0 && idx < MAX_EVENT_TYPE) {
        return _handlers[idx].registered;
    }
    return false;
}

void TfFsmEventAdapter::ClearAllHandlers() {
    for (int i = 0; i < MAX_EVENT_TYPE; ++i) {
        _handlers[i].registered = false;
        _handlers[i].action = nullptr;
        _handlers[i].expected_state = TASK_STATUS_TYPE::UNDEFINED;
    }
}
