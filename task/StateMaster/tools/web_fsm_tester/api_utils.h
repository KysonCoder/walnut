/** @file api_utils.h
 *  @brief 命令校验、状态名映射、/api/status JSON 组装（纯函数，可单测）
 */
#ifndef WEB_FSM_TESTER_API_UTILS_H_
#define WEB_FSM_TESTER_API_UTILS_H_

#include <string>
#include <vector>

#include "shm_watcher.h"   // TaskStatusInfo

namespace web_fsm_tester {

// 序列状态快照（sequence_runner 在 Task 4 提供同样字段的结构；
// 此处独立定义纯数据，避免 api_utils 依赖线程设施）
struct SeqView {
    bool active = false;
    int step = 0;
    int step_count = 8;
    std::string label;
    std::string error;
    bool finished = false;
    bool success = false;
};

bool IsValidStateMsg(const std::string& msg);
const char* StatusName(int status);
std::string BuildStatusJson(bool app_running,
                            const std::vector<TaskStatusInfo>& tasks,
                            const SeqView& seq);

}  // namespace web_fsm_tester

#endif  // WEB_FSM_TESTER_API_UTILS_H_
