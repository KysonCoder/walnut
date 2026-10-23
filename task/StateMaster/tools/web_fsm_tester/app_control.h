/** @file app_control.h
 *  @brief taskframe_launcher 进程托管（搬自 full_app_state_machine_test）
 */
#ifndef WEB_FSM_TESTER_APP_CONTROL_H_
#define WEB_FSM_TESTER_APP_CONTROL_H_

#include <sys/types.h>

namespace web_fsm_tester {

constexpr char kAppDir[] = "/home/user/claude/task/app";
constexpr int kWorkerElfCount = 4;
constexpr const char* kWorkerElfNames[kWorkerElfCount] = {
    "state_master.elf", "planning_task.elf", "servo_task.elf", "fieldbus_task.elf"};

bool process_exists(const char* name);
void kill_process(const char* name);
bool PortInUse(int port);

// fork/exec 托管 taskframe_launcher（cwd=kAppDir，stdin=/dev/null——
// launcher 的 getchar() 收到字符才退出，接空 pipe 会永久阻塞使 SIGTERM 失效）。
// 返回 launcher pid，失败 -1。启动前清理残留进程与 shm。
pid_t LauncherStart();
// SIGTERM 优雅停（最坏 ~10s），兜底 SIGKILL + 清残留 + shm_unlink。
void LauncherStop(pid_t pid);

}  // namespace web_fsm_tester

#endif  // WEB_FSM_TESTER_APP_CONTROL_H_
