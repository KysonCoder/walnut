/**
 * @file multi_process_integration_test.cpp
 * @brief Multi-process integration test for StateMaster + Planning + Servo + Fieldbus
 */

#include <gtest/gtest.h>

#include "state_master.hpp"
#include "planning_task.hpp"
#include "servo_task.hpp"
#include "fieldbus_task.hpp"
#include "task_frame.hpp"
#include "main_fsm.hpp"
#include "tf_event.hpp"
#include "status_manager.hpp"

#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <cstring>
#include <cerrno>

// Externs from TaskFrame
extern void* g_shm_base;
extern size_t g_shm_size;

// Test data externs required when test flags are defined
#ifdef STATE_MASTER_TEST
STATE_MASTER_TEST_DATA _state_master_test_data;
#endif

#ifdef PLANNING_TASK_TEST
PLANNING_TASK_TEST_DATA _planning_task_test_data;
#endif

#ifdef SERVO_TASK_TEST
SERVO_TASK_TEST_DATA _servo_task_test_data;
#endif

#ifdef FIELDBUS_TASK_TEST
FIELDBUS_TASK_TEST_DATA _fieldbus_task_test_data;
#endif

#ifdef TASK_FRAME_TEST
TASK_FRAME_TEST_DATA task_frame_test_data;
#endif

namespace {

// Child process runner: Planning task
void RunPlanningChild() {
    CPlanningTask task;
    char task_name[32] = "planning_task";

    if (!task.OpenSharedMemory()) {
        _exit(1);
    }
    if (!task.InitTask(task_name)) {
        _exit(2);
    }

    while (!task.GetExitFlag()) {
        task.MainLoop();
    }
    _exit(0);
}

// Child process runner: Servo task
void RunServoChild() {
    CServoTask task;
    char task_name[32] = "servo_task";

    if (!task.OpenSharedMemory()) {
        _exit(1);
    }
    if (!task.InitTask(task_name)) {
        _exit(2);
    }

    while (!task.GetExitFlag()) {
        task.MainLoop();
    }
    _exit(0);
}

// Child process runner: Fieldbus task
void RunFieldbusChild() {
    CFieldbusTask task;
    char task_name[32] = "fieldbus_task";

    if (!task.OpenSharedMemory()) {
        _exit(1);
    }
    if (!task.InitTask(task_name)) {
        _exit(2);
    }

    while (!task.GetExitFlag()) {
        task.MainLoop();
    }
    _exit(0);
}

}  // namespace

class MultiProcessIntegrationTest : public ::testing::Test {
 protected:
    CStateMasterTask* _state_master = nullptr;
    pid_t _planning_pid = -1;
    pid_t _servo_pid = -1;
    pid_t _fieldbus_pid = -1;

    void SetUp() override {
        // Create StateMaster as the main task
        _state_master = new CStateMasterTask();
        _state_master->SetMainTask();

        ASSERT_TRUE(_state_master->OpenSharedMemory());

        char task_name[32] = "state_master";
        ASSERT_TRUE(_state_master->InitTask(task_name));

        // Small delay to ensure main task has fully initialized shared memory
        // before children attempt to attach
        usleep(100000);

        // Fork child processes
        _planning_pid = fork();
        ASSERT_GE(_planning_pid, 0) << "fork() for planning failed: " << strerror(errno);
        if (_planning_pid == 0) {
            // Child: planning task
            RunPlanningChild();
            // RunPlanningChild calls _exit() so we never reach here
        }

        _servo_pid = fork();
        ASSERT_GE(_servo_pid, 0) << "fork() for servo failed: " << strerror(errno);
        if (_servo_pid == 0) {
            // Child: servo task
            RunServoChild();
        }

        _fieldbus_pid = fork();
        ASSERT_GE(_fieldbus_pid, 0) << "fork() for fieldbus failed: " << strerror(errno);
        if (_fieldbus_pid == 0) {
            // Child: fieldbus task
            RunFieldbusChild();
        }

        // Wait for all 4 tasks to register
        ASSERT_TRUE(WaitForTaskCount(4, 5000))
            << "Timed out waiting for 4 tasks to register";
    }

    void TearDown() override {
        if (_state_master != nullptr) {
            // Signal all tasks to exit
            _state_master->SetExitFlag(true);
        }

        // Graceful wait for children to exit
        int graceful_status;
        bool planning_exited = false;
        bool servo_exited = false;
        bool fieldbus_exited = false;

        auto graceful_deadline = std::chrono::steady_clock::now()
                                 + std::chrono::seconds(3);

        while (std::chrono::steady_clock::now() < graceful_deadline) {
            if (!planning_exited && _planning_pid > 0) {
                pid_t w = waitpid(_planning_pid, &graceful_status, WNOHANG);
                if (w == _planning_pid) {
                    planning_exited = true;
                }
            }
            if (!servo_exited && _servo_pid > 0) {
                pid_t w = waitpid(_servo_pid, &graceful_status, WNOHANG);
                if (w == _servo_pid) {
                    servo_exited = true;
                }
            }
            if (!fieldbus_exited && _fieldbus_pid > 0) {
                pid_t w = waitpid(_fieldbus_pid, &graceful_status, WNOHANG);
                if (w == _fieldbus_pid) {
                    fieldbus_exited = true;
                }
            }
            if (planning_exited && servo_exited && fieldbus_exited) {
                break;
            }
            usleep(10000);
        }

        // SIGTERM for any still alive
        if (!planning_exited && _planning_pid > 0) {
            kill(_planning_pid, SIGTERM);
        }
        if (!servo_exited && _servo_pid > 0) {
            kill(_servo_pid, SIGTERM);
        }
        if (!fieldbus_exited && _fieldbus_pid > 0) {
            kill(_fieldbus_pid, SIGTERM);
        }

        usleep(100000);

        // SIGKILL for any survivors
        if (!planning_exited && _planning_pid > 0) {
            if (waitpid(_planning_pid, nullptr, WNOHANG) == 0) {
                kill(_planning_pid, SIGKILL);
            }
        }
        if (!servo_exited && _servo_pid > 0) {
            if (waitpid(_servo_pid, nullptr, WNOHANG) == 0) {
                kill(_servo_pid, SIGKILL);
            }
        }
        if (!fieldbus_exited && _fieldbus_pid > 0) {
            if (waitpid(_fieldbus_pid, nullptr, WNOHANG) == 0) {
                kill(_fieldbus_pid, SIGKILL);
            }
        }

        // Reap any remaining zombies
        int status;
        if (_planning_pid > 0) {
            waitpid(_planning_pid, &status, 0);
        }
        if (_servo_pid > 0) {
            waitpid(_servo_pid, &status, 0);
        }
        if (_fieldbus_pid > 0) {
            waitpid(_fieldbus_pid, &status, 0);
        }

        // Clean up shared memory
        delete _state_master;
        _state_master = nullptr;

        shm_unlink(SHARED_MEMORY_NAME);

        // Reset globals
        g_shm_base = nullptr;
        g_shm_size = 0;
    }

    bool WaitForTaskCount(int expected, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

        while (std::chrono::steady_clock::now() < deadline) {
            CStatusManager& sm = _state_master->GetStatusManager();
            if (sm.GetTaskNumber() >= expected) {
                return true;
            }
            usleep(10000);
        }
        return false;
    }

    bool WaitForAllTasksStatus(TASK_STATUS_TYPE expected, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);
        int poll_count = 0;

        while (std::chrono::steady_clock::now() < deadline) {
            // Pump state master's status sync periodically (every ~100ms)
            // so children have time to process events between checks
            if (poll_count % 10 == 0) {
                _state_master->UpdateTaskStatus();
            }
            poll_count++;

            CStatusManager& sm = _state_master->GetStatusManager();
            int task_count = sm.GetTaskNumber();
            int master_task_id = _state_master->GetTaskId();
            int child_count = 0;
            int matched_count = 0;
            for (int i = 0; i < task_count; ++i) {
                // Skip StateMaster itself — only verify sub-tasks
                if (i == master_task_id) {
                    continue;
                }
                int status_value = -1;
                if (sm.GetStatus(i, status_value)) {
                    child_count++;
                    if (status_value == expected) {
                        matched_count++;
                    }
                }
            }
            if (child_count > 0 && matched_count >= child_count) {
                return true;
            }
            usleep(10000);
        }

        // Print snapshot on timeout (skip StateMaster)
        CStatusManager& sm = _state_master->GetStatusManager();
        int task_count = sm.GetTaskNumber();
        int master_task_id = _state_master->GetTaskId();
        printf("WaitForAllTasksStatus timeout snapshot (sub-tasks only):\n");
        for (int i = 0; i < task_count; ++i) {
            if (i == master_task_id) continue;
            int status_value = -1;
            sm.GetStatus(i, status_value);
            printf("  task[%d]: status=%d\n", i, status_value);
        }
        return false;
    }

    bool CheckChildProcessesAlive() {
        int status;
        bool any_crashed = false;

        if (_planning_pid > 0) {
            pid_t w = waitpid(_planning_pid, &status, WNOHANG);
            if (w == _planning_pid) {
                if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
                    printf("Planning child exited with code %d\n", WEXITSTATUS(status));
                    any_crashed = true;
                } else if (WIFSIGNALED(status)) {
                    printf("Planning child killed by signal %d\n", WTERMSIG(status));
                    any_crashed = true;
                }
            }
        }
        if (_servo_pid > 0) {
            pid_t w = waitpid(_servo_pid, &status, WNOHANG);
            if (w == _servo_pid) {
                if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
                    printf("Servo child exited with code %d\n", WEXITSTATUS(status));
                    any_crashed = true;
                } else if (WIFSIGNALED(status)) {
                    printf("Servo child killed by signal %d\n", WTERMSIG(status));
                    any_crashed = true;
                }
            }
        }
        if (_fieldbus_pid > 0) {
            pid_t w = waitpid(_fieldbus_pid, &status, WNOHANG);
            if (w == _fieldbus_pid) {
                if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
                    printf("Fieldbus child exited with code %d\n", WEXITSTATUS(status));
                    any_crashed = true;
                } else if (WIFSIGNALED(status)) {
                    printf("Fieldbus child killed by signal %d\n", WTERMSIG(status));
                    any_crashed = true;
                }
            }
        }
        return !any_crashed;
    }

    bool AdvanceToState(TASK_STATUS_TYPE target) {
        TASK_STATUS_TYPE current = _state_master->GetFSMState();

        // Define transition path from each state to target
        // State order: UNINTIATED -> UNCONFIGED -> MOTOR_OFF -> READY -> RUNNING
        if (current == UNINTIATED && target >= UNCONFIGED) {
            _state_master->DispatchEvent(INIT_EVENT);
            usleep(50000);
            current = _state_master->GetFSMState();
        }
        if (current == UNCONFIGED && target >= MOTOR_OFF) {
            _state_master->DispatchEvent(CONFIG_EVENT);
            usleep(50000);
            current = _state_master->GetFSMState();
        }
        if (current == MOTOR_OFF && target >= READY) {
            _state_master->DispatchEvent(MOTOR_ON_EVENT);
            usleep(50000);
            current = _state_master->GetFSMState();
        }
        if (current == READY && target >= RUNNING) {
            _state_master->DispatchEvent(CONTROL_ON_EVENT);
            usleep(50000);
            current = _state_master->GetFSMState();
        }

        return current == target;
    }
};

TEST_F(MultiProcessIntegrationTest, AllTasksRegistered) {
    // Verify all 4 tasks are registered
    CStatusManager& sm = _state_master->GetStatusManager();
    EXPECT_EQ(sm.GetTaskNumber(), 4);

    // Verify no child process crashed during setup
    EXPECT_TRUE(CheckChildProcessesAlive());

    // Verify all tasks are in UNINTIATED state
    EXPECT_TRUE(WaitForAllTasksStatus(UNINTIATED, 2000))
        << "Not all tasks reached UNINTIATED state";
}

TEST_F(MultiProcessIntegrationTest, GoldenPath_FullFsmSequence) {
    // UNINITIATED -> INIT -> UNCONFIGED
    _state_master->DispatchEvent(INIT_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(UNCONFIGED, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), UNCONFIGED);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // UNCONFIGED -> CONFIG -> MOTOR_OFF
    _state_master->DispatchEvent(CONFIG_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // MOTOR_OFF -> MOTOR_ON -> READY
    _state_master->DispatchEvent(MOTOR_ON_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(READY, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), READY);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // READY -> CONTROL_ON -> RUNNING
    _state_master->DispatchEvent(CONTROL_ON_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), RUNNING);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // RUNNING -> CONTROL_OFF -> READY
    _state_master->DispatchEvent(CONTROL_OFF_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(READY, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), READY);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // READY -> MOTOR_OFF -> MOTOR_OFF
    _state_master->DispatchEvent(MOTOR_OFF_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

// 稳态上抛（2026-09-10）：GoldenPath 到 MOTION 后，父进程把子任务状态字置 ERROR
// （模拟子任务错误），StateMaster 应在 ≤200ms 内进 ERROR 并广播
TEST_F(MultiProcessIntegrationTest, SteadyStateSubTaskErrorEscalates) {
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 2000));

    // 父进程（主任务宿主）直接写子任务状态字模拟子任务进 ERROR
    char planning_name[] = "planning_task";
    ASSERT_TRUE(_state_master->GetStatusManager().SetStatus(planning_name, ERROR));

    // 本 harness 不泵 StateMaster MainLoop——显式调用 UpdateTaskStatus
    // （生产环境 MainLoop 每 64ms 调用一次，等效）
    _state_master->UpdateTaskStatus();

    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    // 子任务收到 ERROR 广播（子任务状态随动进 ERROR）
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 2000));
}

TEST_F(MultiProcessIntegrationTest, Init_Transition) {
    // UNINITIATED + INIT -> UNCONFIGED
    _state_master->DispatchEvent(INIT_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(UNCONFIGED, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), UNCONFIGED);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Config_Transition) {
    // Advance to UNCONFIGED first
    ASSERT_TRUE(AdvanceToState(UNCONFIGED));
    ASSERT_TRUE(WaitForAllTasksStatus(UNCONFIGED, 5000));

    // UNCONFIGED + CONFIG -> MOTOR_OFF
    _state_master->DispatchEvent(CONFIG_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, MotorOn_Transition) {
    // Advance to MOTOR_OFF first
    ASSERT_TRUE(AdvanceToState(MOTOR_OFF));
    ASSERT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));

    // MOTOR_OFF + MOTOR_ON -> READY
    _state_master->DispatchEvent(MOTOR_ON_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(READY, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), READY);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, ControlOn_Transition) {
    // Advance to READY first
    ASSERT_TRUE(AdvanceToState(READY));
    ASSERT_TRUE(WaitForAllTasksStatus(READY, 5000));

    // READY + CONTROL_ON -> RUNNING
    _state_master->DispatchEvent(CONTROL_ON_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), RUNNING);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, ControlOff_Transition) {
    // Advance to RUNNING first
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));

    // RUNNING + CONTROL_OFF -> READY
    _state_master->DispatchEvent(CONTROL_OFF_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(READY, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), READY);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, MotorOff_FromReady_Transition) {
    // Advance to READY first
    ASSERT_TRUE(AdvanceToState(READY));
    ASSERT_TRUE(WaitForAllTasksStatus(READY, 5000));

    // READY + MOTOR_OFF -> MOTOR_OFF
    _state_master->DispatchEvent(MOTOR_OFF_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, MotorOff_FromMotion_Transition) {
    // Advance to RUNNING first (MOTION state in the FSM)
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));

    // RUNNING + MOTOR_OFF -> MOTOR_OFF
    _state_master->DispatchEvent(MOTOR_OFF_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Error_FromUninitiated) {
    // Start from UNINITIATED, dispatch ERROR
    _state_master->DispatchEvent(ERROR_EVENT);

    // All tasks should enter ERROR
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Error_FromUnconfiged) {
    // Advance to UNCONFIGED first
    ASSERT_TRUE(AdvanceToState(UNCONFIGED));
    ASSERT_TRUE(WaitForAllTasksStatus(UNCONFIGED, 5000));

    // Dispatch ERROR event through StateMaster FSM
    _state_master->DispatchEvent(ERROR_EVENT);

    // All tasks should enter ERROR
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Error_FromMotorOff) {
    // Advance to MOTOR_OFF first
    ASSERT_TRUE(AdvanceToState(MOTOR_OFF));
    ASSERT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));

    // Dispatch ERROR event through StateMaster FSM
    _state_master->DispatchEvent(ERROR_EVENT);

    // All tasks should enter ERROR
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Error_FromReady) {
    // Advance to READY first
    ASSERT_TRUE(AdvanceToState(READY));
    ASSERT_TRUE(WaitForAllTasksStatus(READY, 5000));

    // Dispatch ERROR event through StateMaster FSM
    _state_master->DispatchEvent(ERROR_EVENT);

    // All tasks should enter ERROR
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Error_FromMotion) {
    // Advance to RUNNING first
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));

    // Dispatch ERROR event through StateMaster FSM
    _state_master->DispatchEvent(ERROR_EVENT);

    // All tasks should enter ERROR
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Reset_FromError_ToMotorOff) {
    // Enter ERROR state from RUNNING
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));

    _state_master->DispatchEvent(ERROR_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(ERROR, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), ERROR);

    // RESET: ERROR -> MOTOR_OFF (ResetAction returns 0)
    _state_master->DispatchEvent(RESET_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, Reset_FromError_ToUnconfiged) {
    GTEST_SKIP() << "StateMaster::ResetAction() currently returns 0 (stub). "
                 << "Enable when ResetAction supports return 1 -> UNCONFIGED branch.";

    // The test body would be:
    // AdvanceToState(RUNNING);
    // WaitForAllTasksStatus(RUNNING, 5000);
    // DispatchEvent(ERROR_EVENT);
    // WaitForAllTasksStatus(ERROR, 5000);
    // DispatchEvent(RESET_EVENT);  // with ResetAction returning 1
    // WaitForAllTasksStatus(UNCONFIGED, 5000);
    // EXPECT_EQ(GetFSMState(), UNCONFIGED);
}

// EMERGENCY_STOP：基类默认 react 已实现（main_fsm.hpp），全链路可用。

TEST_F(MultiProcessIntegrationTest, EmergencyStop_FromReady) {
    ASSERT_TRUE(AdvanceToState(READY));
    ASSERT_TRUE(WaitForAllTasksStatus(READY, 5000));

    _state_master->DispatchEvent(EMERGENCY_STOP_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(EMERGENCY_STOP, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), EMERGENCY_STOP);
    EXPECT_TRUE(CheckChildProcessesAlive());

    // 人工复位：ESTOP -> MOTOR_OFF
    _state_master->DispatchEvent(RESET_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, EmergencyStop_FromMotion) {
    ASSERT_TRUE(AdvanceToState(RUNNING));
    ASSERT_TRUE(WaitForAllTasksStatus(RUNNING, 5000));

    _state_master->DispatchEvent(EMERGENCY_STOP_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(EMERGENCY_STOP, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), EMERGENCY_STOP);
    EXPECT_TRUE(CheckChildProcessesAlive());

    _state_master->DispatchEvent(RESET_EVENT);
    EXPECT_TRUE(WaitForAllTasksStatus(MOTOR_OFF, 5000));
    EXPECT_EQ(_state_master->GetFSMState(), MOTOR_OFF);
    EXPECT_TRUE(CheckChildProcessesAlive());
}

TEST_F(MultiProcessIntegrationTest, SyncTimeout_TriggersError) {
    GTEST_SKIP() << "Requires a fixture variant with a blocked/non-responsive child task. "
                 << "StateMaster's UpdateTaskStatus() would then timeout after 80 iterations "
                 << "and broadcast ERROR. Implement with a custom SetUp that starts "
                 << "a child that registers but ignores events.";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
