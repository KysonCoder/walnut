/**
 * @file hmi_server.cpp
 * @brief HmiServer 实现
 */

#include "hmi_server.hpp"
#include "hmi_page.hpp"
#include "logger.h"

namespace state_master_hmi {

bool HmiServer::Start() {
    if (hmi_ == nullptr || !hmi_->IsBound()) {
        Log_E("HmiServer::Start: hmi interface not bound\n");
        return false;
    }
    RegisterRoutes();
    server_.set_default_headers(
        {{"Access-Control-Allow-Origin", "*"},
         {"Access-Control-Allow-Methods", "POST, GET, OPTIONS"},
         {"Access-Control-Allow-Headers", "*"},
         // 单页/接口均禁止缓存：页面内嵌在 elf 里，换版后旧 DOM 会拿空占位符
         // 提交（2026-09-09 程序下发"空 JSON"误报的根因）
         {"Cache-Control", "no-store"},
         {"Content-Type", "application/json;charset=utf-8"}});
    server_.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        res.status = 200;
    });
    if (!server_.bind_to_port("0.0.0.0", port_)) {
        Log_E("HmiServer::Start: bind port %d failed\n", port_);
        return false;
    }
    bound_port_ = port_;  // bind_to_port 成功即占用该端口
    running_.store(true);
    thread_ = std::thread([this]() {
        server_.listen_after_bind();
        running_.store(false);
    });
    return true;
}

void HmiServer::Stop() {
    server_.stop();
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

void HmiServer::RegisterRoutes() {
    server_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
        res.set_content(kHmiPageHtml, "text/html;charset=utf-8");
    });
    server_.Get("/queueStatus",
                [this](const httplib::Request&, httplib::Response& res) {
                    SetJsonResponse(res, hmi_->BuildQueueStatusJson());
                });
    server_.Post("/stateMsg",
                 [this](const httplib::Request& q, httplib::Response& r) {
                     HandleStateMsg(q, r);
                 });
    server_.Post("/stateAndPositionFeedback",
                 [this](const httplib::Request& q, httplib::Response& r) {
                     HandleStateAndPositionFeedback(q, r);
                 });
    server_.Post("/executeMovement",
                 [this](const httplib::Request& q, httplib::Response& r) {
                     HandleExecuteMovement(q, r);
                 });
    server_.Post("/executeProgram",
                 [this](const httplib::Request& q, httplib::Response& r) {
                     HandleExecuteProgram(q, r);
                 });
    server_.Post("/stopProgram",
                 [this](const httplib::Request& q, httplib::Response& r) {
                     HandleStopProgram(q, r);
                 });
}

bool HmiServer::ParseJson(const std::string& body, Json::Value& out) {
    Json::Reader reader;
    return reader.parse(body, out);
}

void HmiServer::SetJsonResponse(httplib::Response& res, const Json::Value& body,
                                int status) {
    res.status = status;
    res.set_content(body.toStyledString(), "application/json");
}

void HmiServer::SetErrorResponse(httplib::Response& res, const Json::Value& cmd,
                                 const std::string& info, int status) {
    Json::Value response;
    if (cmd.isMember("cmd")) response["cmd"] = cmd["cmd"];
    if (cmd.isMember("cmdSeq")) response["cmdSeq"] = cmd["cmdSeq"];
    response["result"] = "ERROR";
    response["info"] = info;
    SetJsonResponse(res, response, status);
}

void HmiServer::HandleStateMsg(const httplib::Request& req, httplib::Response& res) {
    Json::Value cmd;
    if (!ParseJson(req.body, cmd)) {
        SetErrorResponse(res, Json::Value(), "JSON parsing error");
        return;
    }
    if (cmd["cmd"].asString() != "REFSTATECHANGE") {
        SetErrorResponse(res, cmd, "unsupported cmd for /stateMsg");
        return;
    }
    // 模式切换消息（msg_manual_mode/msg_auto_mode）：走 CHmiInterface 校验+转发，
    // 拒绝（Planning READY/RUNNING 或未注册）→ 409 + 原因
    const std::string state_msg = cmd["stateMSG"].asString();
    MsgType mt;
    if (StateMsgToMsgType(state_msg, &mt)) {
        std::string err;
        const bool ok = (mt == MsgType::CMD_CALIB_ZERO)
            ? hmi_->SendCalibZeroCommand(&err)
            : hmi_->SendModeCommand(state_msg, &err);
        if (!ok) {
            SetErrorResponse(res, cmd, err, 409);
            return;
        }
        Json::Value response;
        response["cmd"] = "FDBSTATECHANGE";
        response["result"] = "OK!";
        SetJsonResponse(res, response);
        return;
    }
    if (!hmi_->SendFsmEvent(state_msg)) {
        SetErrorResponse(res, cmd, "unknown stateMSG or event ring full");
        return;
    }
    Json::Value response;
    response["cmd"] = "FDBSTATECHANGE";
    response["result"] = "OK!";
    SetJsonResponse(res, response);
}

void HmiServer::HandleStateAndPositionFeedback(const httplib::Request& req,
                                               httplib::Response& res) {
    Json::Value cmd;
    if (!ParseJson(req.body, cmd)) {
        SetErrorResponse(res, Json::Value(), "JSON parsing error");
        return;
    }
    Json::Value response;
    response["cmd"] = "FDBSINGLESOCPOS";
    response["currentState"] = hmi_->GetTaskStatusString();
    response["runningMode"] = hmi_->IsManualMode() ? "manual" : "auto";
    response["socFeedbacks"] = hmi_->BuildSocFeedbacksJson();
    SetJsonResponse(res, response);
}

void HmiServer::HandleExecuteMovement(const httplib::Request& req,
                                      httplib::Response& res) {
    Json::Value cmd;
    if (!ParseJson(req.body, cmd)) {
        SetErrorResponse(res, Json::Value(), "JSON parsing error");
        return;
    }
    const Json::Value& prog = cmd["program"];
    const Json::Value& cv = prog["commandValue"];
    if (!prog.isObject() || !cv.isArray() || cv.size() != 7) {
        SetErrorResponse(res, cmd, "program/commandValue invalid");
        return;
    }
    double command_value[7];
    for (int i = 0; i < 7; i++) command_value[i] = cv[i].asDouble();

    ActionGroupSharedData grp;
    if (!hmi_->BuildManualAction(prog["unitType"].asInt(), prog["motionType"].asInt(),
                                 prog["is_pose_valid"].asBool(), command_value, grp)) {
        SetErrorResponse(res, cmd, "invalid unitType/motionType");
        return;
    }
    if (!hmi_->SubmitManualAction(grp)) {
        SetErrorResponse(res, cmd, "action queue full", 503);
        return;
    }
    Json::Value response;
    response["cmd"] = "FDBMANUALPROGRAM";
    response["Result"] = "Movement executed successfully";
    SetJsonResponse(res, response);
}

void HmiServer::HandleExecuteProgram(const httplib::Request& req,
                                     httplib::Response& res) {
    Json::Value cmd;
    if (!ParseJson(req.body, cmd)) {
        SetErrorResponse(res, Json::Value(), "JSON parsing error");
        return;
    }
    if (cmd["cmd"].asString() != "TESTAUTOPROGRAM" ||
        !cmd["actionGroup"].isArray()) {
        SetErrorResponse(res, cmd, "not a TESTAUTOPROGRAM with actionGroup");
        return;
    }
    int accepted = 0;
    std::string rejected;
    const Json::Value& lines = cmd["actionGroup"];
    for (Json::ArrayIndex i = 0; i < lines.size(); i++) {
        ActionGroupSharedData grp;
        std::string err;
        if (!hmi_->BuildProgramLine(lines[i], static_cast<int>(i), grp, &err)) {
            rejected += "line " + std::to_string(i) + ": " + err + "; ";
            continue;  // 逐行校验：拒绝坏行，不丢整程序
        }
        if (!hmi_->SubmitProgramLine(grp)) {
            rejected += "line " + std::to_string(i) + ": queue full; ";
            continue;
        }
        accepted++;
    }
    Json::Value response;
    response["cmd"] = "FDBMANUALPROGRAM";
    response["Result"] = "Program Push Queue";
    response["accepted"] = accepted;
    if (!rejected.empty()) response["info"] = rejected;
    SetJsonResponse(res, response);
}

void HmiServer::HandleStopProgram(const httplib::Request& req, httplib::Response& res) {
    Json::Value cmd;
    if (!ParseJson(req.body, cmd)) {
        SetErrorResponse(res, Json::Value(), "JSON parsing error");
        return;
    }
    hmi_->SetStopProgram(cmd["StopProgram"].asBool());
    Json::Value response;
    response["cmd"] = "FDBMANUALPROGRAM";
    response["Result"] = "Program Stop";
    SetJsonResponse(res, response);
}

}  // namespace state_master_hmi
