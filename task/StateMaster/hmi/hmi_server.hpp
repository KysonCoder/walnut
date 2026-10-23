/**
 * @file hmi_server.hpp
 * @brief HmiServer — HMI HTTP 传输层（httplib），协议兼容 ref/index.html
 *
 * 只做传输与 JSON 编解码，机器人语义全部委托 CHmiInterface。
 * 非实时线程，不绑核。Start 失败不阻塞主任务（HMI 降级）。
 */

#ifndef __HMI_SERVER_HPP__
#define __HMI_SERVER_HPP__

#include <atomic>
#include <thread>

#include "httplib.h"
#include "json.h"
#include "hmi_interface.hpp"

namespace state_master_hmi {

class HmiServer {
public:
    HmiServer(CHmiInterface* hmi, int port) : hmi_(hmi), port_(port) {}
    ~HmiServer() { Stop(); }

    HmiServer(const HmiServer&) = delete;
    HmiServer& operator=(const HmiServer&) = delete;

    bool Start();   // 绑定端口成功才返回 true；后台线程 listen
    void Stop();
    bool IsRunning() const { return running_.load(); }
    int port() const { return bound_port_; }

private:
    void RegisterRoutes();
    void HandleStateMsg(const httplib::Request& req, httplib::Response& res);
    void HandleStateAndPositionFeedback(const httplib::Request& req, httplib::Response& res);
    void HandleExecuteMovement(const httplib::Request& req, httplib::Response& res);
    void HandleExecuteProgram(const httplib::Request& req, httplib::Response& res);
    void HandleStopProgram(const httplib::Request& req, httplib::Response& res);

    static bool ParseJson(const std::string& body, Json::Value& out);
    static void SetJsonResponse(httplib::Response& res, const Json::Value& body,
                                int status = 200);
    static void SetErrorResponse(httplib::Response& res, const Json::Value& cmd,
                                 const std::string& info, int status = 400);

    CHmiInterface* hmi_;
    int port_;
    int bound_port_ = -1;
    httplib::Server server_;
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace state_master_hmi

#endif  // __HMI_SERVER_HPP__
