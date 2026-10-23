/**
 * @file tf_types.hpp
 * @brief TaskFrame 共享类型的唯一定义点。
 *
 * 所有放入共享内存或跨模块公开的类型集中在此：
 *  - 旧代位掩码 EVENT_TYPE / MESSAGE_DATA（下游源码兼容，数值不可变）
 *  - 新代 EventType / MsgType 枚举
 *  - FEEDBACK_* 反馈结构（尺寸取自 confbase，随 ROBOT_TYPE 编译期切换）
 *  - 任务名注册表 TaskNameRegistry（TfEventManager / TfMsgQueueManager 共用）
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "confbase/robot.hpp"

#define MAX_TASK_NUMBER 32

// ---- 旧代事件位掩码（下游兼容层；禁止改用数值） ----
typedef enum {
  EMPTY_EVENT = 0,
  INIT_EVENT            = 1<<0,
  CONFIG_EVENT          = 1<<1,
  CONTROL_ON_EVENT      = 1<<2,
  CONTROL_OFF_EVENT     = 1<<3,
  MOTOR_ON_EVENT        = 1<<4,
  MOTOR_OFF_EVENT       = 1<<5,
  ERROR_EVENT           = 1<<6,
  PATH_STOP_EVENT       = 1<<7,
  EMERGENCY_STOP_EVENT  = 1<<8,
  SAFE_STOP_EVENT       = 1<<9,
  RESET_EVENT           = 1<<10,
  PAUSE_EVENT           = 1<<11,
  RESUME_EVENT          = 1<<12,
} EVENT_TYPE;

// ---- 新代事件枚举（序数；与 EVENT_TYPE 位掩码之间禁止 static_cast） ----
enum class EventType : uint32_t {
    EMPTY = 0, INIT = 1, CONFIG = 2, CONTROL_ON = 3, CONTROL_OFF = 4,
    MOTOR_ON = 5, MOTOR_OFF = 6, ERROR = 7, PATH_STOP = 8,
    EMERGENCY_STOP = 9, SAFE_STOP = 10, RESET = 11,
    PAUSE = 12, RESUME = 13,
};

constexpr EVENT_TYPE EventTypeToLegacy(EventType type) {
    const uint32_t ordinal = static_cast<uint32_t>(type);
    return (ordinal >= 1 && ordinal <= 13)
               ? static_cast<EVENT_TYPE>(1u << (ordinal - 1))
               : EMPTY_EVENT;
}

constexpr EventType LegacyToEventType(EVENT_TYPE event) {
    switch (event) {
        case INIT_EVENT:           return EventType::INIT;
        case CONFIG_EVENT:         return EventType::CONFIG;
        case CONTROL_ON_EVENT:     return EventType::CONTROL_ON;
        case CONTROL_OFF_EVENT:    return EventType::CONTROL_OFF;
        case MOTOR_ON_EVENT:       return EventType::MOTOR_ON;
        case MOTOR_OFF_EVENT:      return EventType::MOTOR_OFF;
        case ERROR_EVENT:          return EventType::ERROR;
        case PATH_STOP_EVENT:      return EventType::PATH_STOP;
        case EMERGENCY_STOP_EVENT: return EventType::EMERGENCY_STOP;
        case SAFE_STOP_EVENT:      return EventType::SAFE_STOP;
        case RESET_EVENT:          return EventType::RESET;
        case PAUSE_EVENT:          return EventType::PAUSE;
        case RESUME_EVENT:         return EventType::RESUME;
        default:                   return EventType::EMPTY;
    }
}

// ---- 旧代消息类型（下游兼容层） ----
typedef enum {
    MESSAGE_EMPTY = 0,
    MESSAGE_MANUAL_MODE = 1,
    MESSAGE_AUTO_MODE   = 2,
    MESSAGE_PROGRAM_DATA = 3
} TASK_MESSAGE_TYPE;

typedef struct {
    TASK_MESSAGE_TYPE msg_type;
    char msg_data[256];
} MESSAGE_DATA;

// ---- 新代消息枚举 ----
enum class MsgType : uint8_t {
    EMPTY = 0, MANUAL_MODE = 1, AUTO_MODE = 2,
    PROGRAM_ACTION = 10, PROGRAM_TRAJECTORY = 11, PROGRAM_MICROSTEP = 12,
    CONFIG_JOINT_PARAM = 20, CONFIG_SOC_PARAM = 21,
    CMD_JOG = 30, CMD_STOP = 31, CMD_ESTOP = 32,
    CMD_CALIB_ZERO = 33,   // 零位标定捕获（Fieldbus 只读原始编码器并打印片段）
    PROGRAM_PROGRESS = 40,   // Planning→StateMaster：弹出程序组的 action_group_index
                             // （payload = ProgramProgressPayload；2026-10-13 状态恢复）
    START_PROGRAM = 41,   // HMI→StateMaster→Planning：AUTO 程序启动门控开门（2026-10-13）
};

// PROGRAM_PROGRESS 的 payload（shm 消息 POD）。
struct ProgramProgressPayload {
    int32_t group_index;
};

// ---- 反馈数据结构（shm 驻留；尺寸来自 confbase 编译期常量） ----
typedef struct {
    double position_ref;
    double position_feedback;
    double filtered_position;
    double speed_ref;
    double speed_feedback;
    double filtered_speed;
    double torque_ref;
    double torque_feedback;
    double pos_torque_lim;
    double neg_torque_lim;
    double filtered_torque_ref; /* Needed in coldet */
} FEEDBACK_JOINT;

typedef struct {
    FEEDBACK_JOINT joints[robot_config::kRobotMaxJoints];
} FEEDBACK_ALL_JOINTS;

typedef struct {
    double x;
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
} FEEDBACK_POSE;

typedef struct {
    FEEDBACK_POSE soc_pose[robot_config::kRobotMaxUnits];
} FEEDBACK_ALL_POSE;

typedef struct {
    double force_x;
    double force_y;
    double force_z;
    double torque_x;
    double torque_y;
    double torque_z;
} FEEDBACK_FORCE;  /* values only valid with force sensor */

typedef struct {
    FEEDBACK_FORCE soc_force[robot_config::kRobotMaxUnits];
} FEEDBACK_ALL_FORCE;

static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_JOINTS>,
              "feedback structs must be shm-safe");
static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_POSE>,
              "feedback structs must be shm-safe");
static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_FORCE>,
              "feedback structs must be shm-safe");
static_assert(std::is_trivially_copyable_v<MESSAGE_DATA>,
              "MESSAGE_DATA must be shm-safe");

// ---- 任务名注册表（shm 驻留；写入必须持有 GlobalData 全局注册互斥锁） ----
struct TaskNameSlot {
    std::atomic<uint8_t> ready{0};  // 1 = name 已完整写入
    char name[32];
};

struct TaskNameRegistry {
    std::atomic<uint32_t> count{0};  // 已使用槽位上界（单调不减）
    TaskNameSlot slots[MAX_TASK_NUMBER];
};
