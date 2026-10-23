/**
 * @file hmi_interface.cpp
 * @brief CHmiInterface 实现
 */

#include "hmi_interface.hpp"
#include <cstring>
#include "logger.h"
#include "confbase/robot.hpp"

namespace state_master_hmi {

bool StateMsgToEventType(const std::string& msg, EventType* out) {
    if (out == nullptr) return false;
    if (msg == "msg_initialize")       *out = EventType::INIT;
    else if (msg == "msg_config")      *out = EventType::CONFIG;
    else if (msg == "msg_motor_on")    *out = EventType::MOTOR_ON;
    else if (msg == "msg_motor_off")   *out = EventType::MOTOR_OFF;
    else if (msg == "msg_control_on")  *out = EventType::CONTROL_ON;
    else if (msg == "msg_control_off") *out = EventType::CONTROL_OFF;
    else if (msg == "msg_error")       *out = EventType::ERROR;
    else if (msg == "msg_reset_ok")    *out = EventType::RESET;
    else if (msg == "msg_estop")       *out = EventType::EMERGENCY_STOP;
    else return false;
    return true;
}

bool StateMsgToMsgType(const std::string& msg, MsgType* out) {
    if (out == nullptr) return false;
    if (msg == "msg_manual_mode")      *out = MsgType::MANUAL_MODE;
    else if (msg == "msg_auto_mode")   *out = MsgType::AUTO_MODE;
    else if (msg == "msg_calib_zero")  *out = MsgType::CMD_CALIB_ZERO;
    else return false;
    return true;
}

bool CHmiInterface::SendCalibZeroCommand(std::string* err) {
    if (task_ == nullptr) {
        if (err) *err = "not bound";
        return false;
    }
    // 零位标定捕获（2026-09-10 设计 §3.2）：只读操作，任意状态可发
    char fieldbus_name[] = "fieldbus_task";  // GetStatus 接口取 char*（不修改内容）
    int st = -1;
    if (!task_->GetStatusManager().GetStatus(fieldbus_name, st)) {
        if (err) *err = "fieldbus task not found";
        return false;
    }
    if (!task_->GetMsgQueueManager().Send("fieldbus_task", MsgType::CMD_CALIB_ZERO)) {
        if (err) *err = "fieldbus msg queue full or missing";
        return false;
    }
    return true;
}

bool CHmiInterface::SendModeCommand(const std::string& state_msg, std::string* err) {
    MsgType mt;
    if (!StateMsgToMsgType(state_msg, &mt)) {
        if (err) *err = "not a mode message";
        return false;
    }
    if (task_ == nullptr) {
        if (err) *err = "not bound";
        return false;
    }
    // 切换约束（2026-09-08 手动模式设计 §2）：Planning 在 READY/RUNNING 禁切。
    char planning_name[] = "planning_task";  // GetStatus 接口取 char*（不修改内容）
    int st = -1;
    if (!task_->GetStatusManager().GetStatus(planning_name, st)) {
        if (err) *err = "planning task not found";
        return false;
    }
    if (st == READY || st == RUNNING) {
        if (err) *err = "planning in READY/MOTION, switch mode after MOTOR_OFF";
        return false;
    }
    if (!task_->GetMsgQueueManager().Send("planning_task", mt)) {
        if (err) *err = "planning msg queue full or missing";
        return false;
    }
    task_->SetRunningModeManual(mt == MsgType::MANUAL_MODE);
    return true;
}

bool MapWireMotionType(int wire, ACTION_MOTION_TYPE* out) {
    if (out == nullptr) return false;
    switch (wire) {
        case 1: *out = ACTION_MOTION_TYPE::ACTION_JOINT; return true;  // 线上 1 = 关节
        case 2: *out = ACTION_MOTION_TYPE::ACTION_LINE;  return true;  // 线上 2 = 直线
        default:
            Log_W("MapWireMotionType: unknown wire motionType %d, rejected\n", wire);
            return false;
    }
}

bool CHmiInterface::BindTask(CStateMasterTask* task) {
    if (task == nullptr) {
        Log_E("CHmiInterface::BindTask: null task\n");
        return false;
    }
    task_ = task;
    return true;
}

bool CHmiInterface::SendFsmEvent(const std::string& state_msg) {
    EventType type;
    if (!StateMsgToEventType(state_msg, &type)) {
        Log_E("CHmiInterface: unknown stateMSG '%s'\n", state_msg.c_str());
        return false;
    }
    if (task_ == nullptr) {
        Log_E("CHmiInterface: not bound\n");
        return false;
    }
    // HTTP 线程是 StateMaster 自身事件环的唯一生产者（主循环只消费），SPSC 成立
    if (!task_->GetEventManager().PublishEvent(task_->GetTaskId(), type)) {
        Log_E("CHmiInterface: PublishEvent failed (ring full?)\n");
        return false;
    }
    return true;
}

std::string CHmiInterface::GetTaskStatusString() {
    if (task_ == nullptr) return "sta_unknown";
    int status = 0;
    if (!task_->GetStatusManager().GetStatus(task_->GetTaskId(), status)) {
        return "sta_unknown";
    }
    switch (static_cast<TASK_STATUS_TYPE>(status)) {
        case UNINTIATED: return "sta_uninitiated";
        case UNCONFIGED: return "sta_unconfiged";
        case MOTOR_OFF:  return "sta_motor_off";
        case READY:      return "sta_ready";
        case RUNNING:    return "sta_motion";
        case EMERGENCY_STOP: return "sta_estop";
        case SAFE_STOP:  return "sta_safe_stop";
        case ERROR:      return "sta_error";
        default:         return "sta_unknown";
    }
}

bool CHmiInterface::GetJointFeedback(FEEDBACK_ALL_JOINTS& out) {
    if (task_ == nullptr) return false;
    return task_->GetFeedbackManager().GetJoints(out);
}

Json::Value CHmiInterface::BuildQueueStatusJson() {
    Json::Value root;
    root["currentState"] = GetTaskStatusString();
    root["runningMode"] = IsManualMode() ? "manual" : "auto";
    if (task_ == nullptr) return root;

    auto* manual = task_->GetManualActionList();
    auto* program = task_->GetProgramActionList();
    auto* traj = task_->GetTrajectoryList();
    auto* micro = task_->GetMicroList();
    // Size() 读双原子索引，第三方只读对诊断足够（非同步快照）
    root["manualList"]["size"] = manual != nullptr ? static_cast<int>(manual->Size()) : -1;
    root["manualList"]["capacity"] = 8;
    root["programList"]["size"] = program != nullptr ? static_cast<int>(program->Size()) : -1;
    root["programList"]["capacity"] = 32;
    root["trajList"]["size"] = traj != nullptr ? static_cast<int>(traj->Size()) : -1;
    root["trajList"]["capacity"] = 64;
    root["microList"]["size"] = micro != nullptr ? static_cast<int>(micro->Size()) : -1;
    root["microList"]["capacity"] = 16;

    FEEDBACK_ALL_JOINTS fb{};
    Json::Value joints(Json::arrayValue);
    if (task_->GetFeedbackManager().GetJoints(fb)) {
        for (uint32_t i = 0; i < robot_config::kRobotMaxJoints; ++i)
            joints.append(fb.joints[i].position_feedback);
    }
    root["feedback"] = joints;
    return root;
}

Json::Value CHmiInterface::BuildSocFeedbacksJson() {    Json::Value soc_feedbacks(Json::arrayValue);
    if (task_ == nullptr) return soc_feedbacks;

    FEEDBACK_ALL_JOINTS fb{};
    const bool has_fb = GetJointFeedback(fb);

    auto& store = robot_config::RobotConfigStore::Instance();
    const robot_config::HumanoidGlobalJointMap* map = store.GetGlobalJointMap();

    for (int u = 0; u < static_cast<int>(robot_config::kRobotMaxUnits); u++) {
        auto unit_type = static_cast<robot_config::RobotSocUnitType>(u);
        const robot_config::HumanoidUnitConfig* unit = store.GetUnit(unit_type);
        if (unit == nullptr || unit->num_joints <= 0 || map == nullptr) continue;

        Json::Value soc_data;
        for (int i = 0; i < unit->num_joints; i++) {
            int32_t phys = unit->logical_to_physical[i];
            if (phys < 0) continue;
            int32_t logical = map->physical_to_logical[phys];
            double pos = 0.0;
            if (has_fb && logical >= 0 &&
                logical < static_cast<int32_t>(robot_config::kRobotMaxJoints)) {
                pos = fb.joints[logical].position_feedback;
            }
            soc_data["jointAngle"]["J" + std::to_string(i + 1)] = pos;
        }
        soc_feedbacks.append(soc_data);
    }
    return soc_feedbacks;
}

bool CHmiInterface::BuildManualAction(int unit_type, int motion_type,
                                      bool is_pose_valid,
                                      const double command_value[7],
                                      ActionGroupSharedData& out) {
    if (unit_type < 0 || unit_type >= MAX_NO_OF_SOC || command_value == nullptr) {
        Log_E("CHmiInterface::BuildManualAction: bad unit_type=%d\n", unit_type);
        return false;
    }
    out = ActionGroupSharedData{};
    ActionData& a = out.actions[unit_type];
    if (!MapWireMotionType(motion_type, &a.motion_type)) {
        Log_E("CHmiInterface::BuildManualAction: invalid motionType %d\n", motion_type);
        return false;
    }
    a.unit_type = static_cast<SOC_UNIT_TYPE>(unit_type);
    a.valid_start_target = false;

    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    if (is_pose_valid) {
        t.pose.pos.x = command_value[0];
        t.pose.pos.y = command_value[1];
        t.pose.pos.z = command_value[2];
        t.pose.orient.q1 = command_value[3];  // qx
        t.pose.orient.q2 = command_value[4];  // qy
        t.pose.orient.q3 = command_value[5];  // qz
        t.pose.orient.q0 = command_value[6];  // qw
        t.is_position_valid = true;
        t.is_orientation_valid = true;
        t.is_joints_valid = false;
    } else {
        for (int i = 0; i < 7; i++) t.joints[i] = command_value[i];
        t.joint_number = 7;
        t.is_joints_valid = true;
    }
    // 速度字段不填（保持清零，同 ref 手动动作语义）
    out.soc_enable[unit_type] = true;
    return true;
}

bool CHmiInterface::BuildProgramLine(const Json::Value& line, int line_index,
                                     ActionGroupSharedData& out, std::string* err) {
    if (!line.isArray() || line.empty()) {
        if (err) *err = "program line is not a non-empty array";
        return false;
    }
    out = ActionGroupSharedData{};
    out.action_group_index = line_index;

    // 注意：不能复用 BuildManualAction（它会清零 out），逐元素内联填充
    for (Json::ArrayIndex i = 0; i < line.size(); i++) {
        const Json::Value& item = line[i];
        int unit_type = item["unitType"].asInt();
        if (unit_type < 0 || unit_type >= MAX_NO_OF_SOC) {
            if (err) *err = "unitType out of range: " + std::to_string(unit_type);
            return false;
        }
        const Json::Value& cv = item["commandValue"];
        if (!cv.isArray() || cv.size() != 7) {
            if (err) *err = "commandValue must have 7 elements";
            return false;
        }

        ActionData& a = out.actions[unit_type];
        a = ActionData{};
        const int wire_mt = item["motionType"].asInt();
        if (!MapWireMotionType(wire_mt, &a.motion_type)) {
            if (err) *err = "unknown motionType " + std::to_string(wire_mt) +
                            " at item " + std::to_string(i);
            return false;
        }
        a.unit_type = static_cast<SOC_UNIT_TYPE>(unit_type);
        const std::string name = item["name"].asString();
        strncpy(a.name, name.c_str(), ACTION_INSTANCE_NAME_LEN);
        a.name[ACTION_INSTANCE_NAME_LEN] = '\0';
        a.action_speed.tcp_m_s = item["speed"]["tcp_m_s"].asDouble();
        a.action_speed.joint_rad_s = item["speed"]["joint_rad_s"].asDouble();
        // ori_rad_s/ext_rad_s 在 ACTION_SPEED_DATA 中无对应字段，本期忽略

        TargetData& t = a.end_target;
        t.unit_type = a.unit_type;
        if (item["is_pose_valid"].asBool()) {
            t.pose.pos.x = cv[0].asDouble();
            t.pose.pos.y = cv[1].asDouble();
            t.pose.pos.z = cv[2].asDouble();
            t.pose.orient.q1 = cv[3].asDouble();
            t.pose.orient.q2 = cv[4].asDouble();
            t.pose.orient.q3 = cv[5].asDouble();
            t.pose.orient.q0 = cv[6].asDouble();
            t.is_position_valid = true;
            t.is_orientation_valid = true;
            t.is_joints_valid = false;
        } else {
            for (int j = 0; j < 7; j++) t.joints[j] = cv[j].asDouble();
            t.joint_number = 7;
            t.is_joints_valid = true;
        }
        out.soc_enable[unit_type] = true;
    }
    return true;
}

bool CHmiInterface::SubmitManualAction(const ActionGroupSharedData& data) {
    if (task_ == nullptr || task_->GetManualActionList() == nullptr) return false;
    if (!task_->GetManualActionList()->TryPush(data)) {
        Log_E("CHmiInterface: manual action queue full\n");
        return false;
    }
    return true;
}

bool CHmiInterface::SubmitProgramLine(const ActionGroupSharedData& data) {
    if (task_ == nullptr || task_->GetProgramActionList() == nullptr) return false;
    if (!task_->GetProgramActionList()->TryPush(data)) {
        Log_E("CHmiInterface: program action queue full\n");
        return false;
    }
    return true;
}

}  // namespace state_master_hmi
