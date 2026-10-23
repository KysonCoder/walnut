#pragma once
#include <functional>
#include "tf_event.hpp"
#include "status_manager.hpp"

class CTaskFrame;

class TfFsmEventAdapter {
public:
    using ActionFunc = std::function<int(CTaskFrame*)>;

    TfFsmEventAdapter();
    void RegisterEventHandler(EventType event, ActionFunc action, TASK_STATUS_TYPE expected_state);
    void UnregisterEventHandler(EventType event);
    void OnEventReceived(const Event& event, CTaskFrame* task);
    TASK_STATUS_TYPE GetExpectedState(EventType event) const;
    bool HasHandler(EventType event) const;
    void ClearAllHandlers();

private:
    static constexpr int MAX_EVENT_TYPE = 64;
    struct HandlerEntry {
        ActionFunc action;
        TASK_STATUS_TYPE expected_state;
        bool registered;
    };
    HandlerEntry _handlers[MAX_EVENT_TYPE];
};
