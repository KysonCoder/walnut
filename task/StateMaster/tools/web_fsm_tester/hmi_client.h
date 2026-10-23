/** @file hmi_client.h
 *  @brief 转发状态命令到 StateMaster HMI（POST 127.0.0.1:8080/stateMsg）
 */
#ifndef WEB_FSM_TESTER_HMI_CLIENT_H_
#define WEB_FSM_TESTER_HMI_CLIENT_H_

#include <string>

namespace web_fsm_tester {

class HmiClient {
public:
    explicit HmiClient(int port = 8080) : port_(port) {}
    // 成功（200 + "OK!"）仅代表事件已入 StateMaster 事件环，不代表切换完成
    bool SendFsmCommand(const std::string& state_msg, std::string* err);

    // POST /stateAndPositionFeedback {cmd:"REFSINGLESOCPOS"}；
    // HTTP 200 时把响应体原样写入 json_out（透明中继，不解析）。
    bool GetPositionFeedback(std::string* json_out, std::string* err);

private:
    int port_;
};

}  // namespace web_fsm_tester

#endif  // WEB_FSM_TESTER_HMI_CLIENT_H_
