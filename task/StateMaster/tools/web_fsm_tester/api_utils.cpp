#include "api_utils.h"

#include "json.h"

namespace web_fsm_tester {

bool IsValidStateMsg(const std::string& msg) {
    static const char* kValid[] = {
        "msg_initialize", "msg_config", "msg_motor_on", "msg_motor_off",
        "msg_control_on", "msg_control_off", "msg_error", "msg_reset_ok",
        "msg_estop"};
    for (const char* v : kValid) {
        if (msg == v) return true;
    }
    return false;
}

const char* StatusName(int status) {
    switch (status) {
        case UNINTIATED: return "UNINITIATED";
        case UNCONFIGED: return "UNCONFIGED";
        case MOTOR_OFF:  return "MOTOR_OFF";
        case READY:      return "READY";
        case RUNNING:    return "RUNNING";
        case EMERGENCY_STOP: return "EMERGENCY_STOP";
        case ERROR:      return "ERROR";
        default:         return "UNKNOWN";
    }
}

std::string BuildStatusJson(bool app_running,
                            const std::vector<TaskStatusInfo>& tasks,
                            const SeqView& seq) {
    Json::Value root;
    root["running"] = app_running;
    Json::Value arr(Json::arrayValue);
    for (const auto& t : tasks) {
        Json::Value e;
        e["name"] = t.name;
        e["status"] = t.status;
        e["statusName"] = StatusName(t.status);
        arr.append(e);
    }
    root["tasks"] = arr;
    Json::Value s;
    s["active"] = seq.active;
    s["step"] = seq.step;
    s["stepCount"] = seq.step_count;
    s["label"] = seq.label;
    s["error"] = seq.error;
    s["finished"] = seq.finished;
    s["success"] = seq.success;
    root["seq"] = s;
    Json::StreamWriterBuilder wb;
    return Json::writeString(wb, root);
}

}  // namespace web_fsm_tester
