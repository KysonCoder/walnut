#include "hmi_client.h"

#include "httplib.h"

namespace web_fsm_tester {

bool HmiClient::SendFsmCommand(const std::string& state_msg, std::string* err) {
    httplib::Client cli("127.0.0.1", port_);
    cli.set_connection_timeout(2, 0);
    cli.set_read_timeout(5, 0);
    std::string body = std::string(R"({"cmd":"REFSTATECHANGE","stateMSG":")")
                       + state_msg + "\"}";
    auto res = cli.Post("/stateMsg", body, "application/json");
    if (!res) { *err = "no response (connection failed)"; return false; }
    if (res->status != 200 || res->body.find("OK!") == std::string::npos) {
        *err = "HTTP " + std::to_string(res->status) + ": " + res->body;
        return false;
    }
    return true;
}

bool HmiClient::GetPositionFeedback(std::string* json_out, std::string* err) {
    httplib::Client cli("127.0.0.1", port_);
    cli.set_connection_timeout(2, 0);
    cli.set_read_timeout(5, 0);
    auto res = cli.Post("/stateAndPositionFeedback",
                        R"({"cmd":"REFSINGLESOCPOS"})", "application/json");
    if (!res) { *err = "no response (connection failed)"; return false; }
    if (res->status != 200) {
        *err = "HTTP " + std::to_string(res->status);
        return false;
    }
    *json_out = res->body;
    return true;
}

}  // namespace web_fsm_tester
