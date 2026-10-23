/**
 * @file hmi_interface.hpp
 * @brief CHmiInterface — HMI 命令与 StateMaster 之间的语义适配层
 *
 * 移植自 ref/application/src/hmi/hmi_interface.cpp，适配新框架：
 * TfEventManager / CStatusManager / TfFeedbackManager / ActionGroupSharedData。
 * 不在实时路径上（允许 std::string/jsoncpp），但 FSM dispatch 保持主循环单线程：
 * 本类只向 StateMaster 自身事件环注入事件。
 */

#ifndef __HMI_INTERFACE_HPP__
#define __HMI_INTERFACE_HPP__

#include <atomic>
#include <string>

#include "json.h"
#include "state_master.hpp"
#include "tf_types.hpp"
#include "action/actiondata.hpp"

namespace state_master_hmi {

// HMI 状态字符串 → EventType 映射（独立可测）。false=未知字符串。
bool StateMsgToEventType(const std::string& msg, EventType* out);

// HMI 模式字符串 → MsgType 映射（独立可测）。false=非模式字符串。
// （2026-09-08 手动模式设计：模式切换走消息环，不进事件 FSM）
bool StateMsgToMsgType(const std::string& msg, MsgType* out);

// 线上协议 motionType（1=关节 2=直线）→ 库枚举 ACTION_MOTION_TYPE。
// 注意与库枚举编号不同（库: LINE=1, CIRCLE=2, JOINT=3）——禁止 static_cast 直转。
// 2026-09-03 起非法值返回 false（ACTION_NULL 新语义 = 该 SOC 不动，
// 静默放行会让操作者误以为已下发动作）。
bool MapWireMotionType(int wire, ACTION_MOTION_TYPE* out);

class CHmiInterface {
public:
    CHmiInterface() = default;

    // 绑定 StateMaster 任务（须在 InitTask 完成后调用）。false=空指针。
    bool BindTask(CStateMasterTask* task);
    bool IsBound() const { return task_ != nullptr; }

    // 状态命令：映射并注入 StateMaster 自身事件环（HTTP 线程是该环唯一生产者，SPSC）。
    bool SendFsmEvent(const std::string& state_msg);

    // 模式切换：Planning 在 READY/RUNNING（或未注册）时拒绝（false，err 含原因）；
    // 成功则经 msg 环转发并更新任务侧模式记录。
    bool SendModeCommand(const std::string& state_msg, std::string* err);

    // 零位标定捕获：转发 Fieldbus（只读操作，任意状态可发；fieldbus 未注册则拒绝）
    bool SendCalibZeroCommand(std::string* err);

    // 当前运行模式（供反馈 JSON 显示；未绑定视为 manual 默认）
    bool IsManualMode() const { return task_ == nullptr || task_->IsManualMode(); }

    std::string GetTaskStatusString();
    bool GetJointFeedback(FEEDBACK_ALL_JOINTS& out);

    // 按 unit 分组的关节反馈 JSON：[{"jointAngle":{"J1":..}}, ...]（跳过未用 unit 槽）
    Json::Value BuildSocFeedbacksJson();

    // 队列水位诊断 JSON（2026-09-08 部署排障）：四条环 Size + 状态/模式 + 全关节反馈
    Json::Value BuildQueueStatusJson();

    // 手动动作 JSON 字段 → ActionGroupSharedData（false=参数非法）
    bool BuildManualAction(int unit_type, int motion_type, bool is_pose_valid,
                           const double command_value[7], ActionGroupSharedData& out);

    // 程序行（actionGroup 二维数组的内层数组）→ ActionGroupSharedData
    bool BuildProgramLine(const Json::Value& line, int line_index,
                          ActionGroupSharedData& out, std::string* err);

    bool SubmitManualAction(const ActionGroupSharedData& data);  // 队列满→false
    bool SubmitProgramLine(const ActionGroupSharedData& data);

    void SetStopProgram(bool stop) { stop_program_.store(stop); }
    bool IsStopProgram() const { return stop_program_.load(); }

private:
    CStateMasterTask* task_ = nullptr;
    std::atomic<bool> stop_program_{false};
};

}  // namespace state_master_hmi

#endif  // __HMI_INTERFACE_HPP__
