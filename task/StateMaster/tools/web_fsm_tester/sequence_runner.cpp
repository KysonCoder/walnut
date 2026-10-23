#include "sequence_runner.h"

namespace web_fsm_tester {

const std::vector<SeqStep>& SequenceSteps() {
    static const std::vector<SeqStep> kSteps = {
        {"msg_initialize",  UNCONFIGED, 25000, "INIT"},
        {"msg_config",      MOTOR_OFF,  10000, "CONFIG"},
        {"msg_motor_on",    READY,      15000, "MOTOR_ON"},
        {"msg_control_on",  RUNNING,    10000, "CONTROL_ON"},
        {"msg_control_off", READY,      10000, "CONTROL_OFF"},
        {"msg_motor_off",   MOTOR_OFF,  10000, "MOTOR_OFF"},
        {"msg_error",       ERROR,      10000, "ERROR"},
        {"msg_reset_ok",    MOTOR_OFF,  10000, "RESET"},
    };
    return kSteps;
}

void SequenceRunner::SetLabel(const std::string& s) {
    std::lock_guard<std::mutex> lk(mu_);
    label_ = s;
}

void SequenceRunner::SetError(const std::string& s) {
    std::lock_guard<std::mutex> lk(mu_);
    error_ = s;
}

bool SequenceRunner::Start(ShmStatusWatcher* watcher, HmiClient* hmi) {
    if (active_.load()) return false;
    watcher_ = watcher;
    hmi_ = hmi;
    stop_requested_.store(false);
    step_.store(0);
    {
        std::lock_guard<std::mutex> lk(mu_);
        label_.clear();
        error_.clear();
        finished_ = false;
        success_ = false;
    }
    active_.store(true);
    thread_ = std::thread(&SequenceRunner::Run, this);
    return true;
}

void SequenceRunner::RequestStop() { stop_requested_.store(true); }

void SequenceRunner::Shutdown() {
    RequestStop();
    if (thread_.joinable()) thread_.join();
}

SeqView SequenceRunner::View() {
    SeqView v;
    v.active = active_.load();
    v.step = step_.load();
    v.step_count = (int)SequenceSteps().size();
    std::lock_guard<std::mutex> lk(mu_);
    v.label = label_;
    v.error = error_;
    v.finished = finished_;
    v.success = success_;
    return v;
}

void SequenceRunner::Run() {
    const auto& steps = SequenceSteps();
    for (int i = 0; i < (int)steps.size(); i++) {
        if (stop_requested_.load()) break;
        step_.store(i + 1);
        SetLabel(steps[i].label);
        std::string err;
        if (!hmi_->SendFsmCommand(steps[i].msg, &err)) {
            SetError(std::string("HMI rejected ") + steps[i].msg + ": " + err);
            break;
        }
        bool synced = watcher_->WaitForAllTasks(
            steps[i].expected_status, steps[i].timeout_ms,
            [this] { return stop_requested_.load(); });
        if (!synced) {
            if (!stop_requested_.load()) {
                SetError(std::string("sync timeout at ") + steps[i].label +
                         ": " + watcher_->Snapshot());
            }
            break;
        }
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        success_ = error_.empty() && !stop_requested_.load()
                   && step_.load() == (int)steps.size();
        finished_ = true;
    }
    active_.store(false);
}

}  // namespace web_fsm_tester
