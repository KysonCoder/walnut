/**
 * @file action_fsm_types.hpp
 * @brief actionFSM 基础 POD 类型：常量、枚举、错误码、信号/触发/状态/上下文
 * @date 2026-08-27
 *
 * 全部类型为纯 POD（standard_layout + trivially_copyable），
 * 可放共享内存（utilbase GlobalData<T,N> 池，其 is_pod 断言禁用 std::atomic）。
 * 跨进程原子访问由 View 层（CActionFsm / FsmSignalQueue 成员函数）
 * 经 GCC __atomic 内建完成，本头文件不含任何 std::atomic 成员。
 */
#ifndef __ACTION_FSM_TYPES_H__
#define __ACTION_FSM_TYPES_H__

#include <cstdint>
#include <type_traits>

// ============================================================================
// 容量上限（固定数组，CLAUDE.md §3.2）
// ============================================================================
constexpr int kFsmMaxStates         = 16;
constexpr int kFsmMaxTransitions    = 64;
constexpr int kFsmSignalQueueDepth  = 16;   // 必须为 2 的幂
constexpr int kFsmMaxTriggers       = 32;
constexpr int kFsmMaxFlags          = 32;
constexpr int kFsmMaxSignalsPerStep = 4;    // 每插补周期队列消费上限，防洪流
constexpr int kFsmStateNameLen      = 16;
constexpr int kFsmMaxJoints         = 7;    // 与 SOC_MAX_JOINT_NUMBER 一致

// 布局校验（attach 方检查 magic/version 一致）
constexpr uint32_t kFsmMagic   = 0x4146534D;  // 'AFSM'
constexpr uint32_t kFsmVersion = 1;

// override 钳位范围 [0, 2] 即 0%~200%
constexpr double kFsmOverrideMin = 0.0;
constexpr double kFsmOverrideMax = 2.0;

// ============================================================================
// 错误码（全大写枚举，CLAUDE.md §2.1）
// LoadConfig 失败返回 FSM_CONFIG_ERR(code, idx)：负值，/65536 得错误码，%65536 得首错索引
// ============================================================================
enum FsmError {
    FSM_OK = 0,
    FSM_ERR_NOT_ATTACHED        = -1,
    FSM_ERR_QUEUE_FULL          = -2,
    FSM_ERR_BAD_ARG             = -3,
    FSM_ERR_CONFIG_DUP_STATE    = -10,
    FSM_ERR_CONFIG_UNKNOWN_STATE= -11,
    FSM_ERR_CONFIG_TERMINAL_EDGE= -12,
    FSM_ERR_CONFIG_BAD_FLAG_ID  = -13,
    FSM_ERR_CONFIG_BAD_INITIAL  = -14,
    FSM_ERR_CONFIG_BAD_TRIGGER  = -15
};

#define FSM_CONFIG_ERR(code, idx) ((code) * 65536 - (int)(idx))
#define FSM_CONFIG_ERR_CODE(ret)  ((ret) / 65536)
#define FSM_CONFIG_ERR_IDX(ret)   (-((ret) % 65536))

// ============================================================================
// 信号与触发类型
// ============================================================================
enum class FsmSignalType : uint8_t {
    NONE = 0,
    START,         // 队列
    STOP,          // 队列
    RESET,         // 队列：回初始状态；清 fired 与诊断计数（flags 归外部消费方清除）
    SET_OVERRIDE,  // 队列：param = 速度覆盖因子
    ESTOP,         // 紧急槽位（写覆盖，Step 最先消费）
    USER_0, USER_1, USER_2, USER_3,
    USER_4, USER_5, USER_6, USER_7,
    SIGNAL_TYPE_COUNT
};

enum class FsmTriggerType : uint8_t {
    NONE = 0,
    PATH_POS,   // norm_path_pos >= threshold
    TIME,       // elapsed_time >= threshold
    JOINT_POS,  // joint_mask 选中关节全部 ctx.joints[j] >= joints_threshold[j]
    CART_POS    // 三轴 |cart_pos[a] - cart_target[a]| <= cart_tolerance
};

// ============================================================================
// POD 结构
// ============================================================================
struct FsmSignal {
    FsmSignalType type;
    uint8_t  reserved[7];
    double   param;      // SET_OVERRIDE 因子；USER_x 附带参数
    uint32_t seq;        // 注入方序号（调试/丢序检测）
    uint32_t reserved2;
};

struct FsmTrigger {
    uint8_t enabled;
    FsmTriggerType type;
    uint8_t joint_mask;   // bit j = 关节 j 参与 JOINT_POS 判定
    uint8_t flag_id;      // 命中置位的输出标志 [0, kFsmMaxFlags)
    double threshold;     // PATH_POS / TIME 阈值
    double joints_threshold[kFsmMaxJoints];
    double cart_target[3];
    double cart_tolerance;
    uint8_t fired;        // 单次触发；RESET 转移或从 terminal 出发的转移时重新武装
    uint8_t reserved[7];
};

struct FsmFlag {
    uint8_t id;
    uint8_t reserved[7];
    uint32_t set;         // 0/1；__atomic 访问
    uint32_t reserved2;
    uint64_t cycle_stamp; // 置位时的插补周期；__atomic 访问
};

struct FsmState {
    uint8_t id;
    uint8_t is_terminal;    // 终止状态（DONE/ABORTED），不允许有出边
    uint8_t is_error;       // 错误状态
    uint8_t permits_motion; // 该状态是否允许路径推进
    char name[kFsmStateNameLen];
};

struct FsmTransition {
    uint8_t from_state;
    uint8_t signal;         // FsmSignalType 值
    uint8_t to_state;
    uint8_t reserved;
};

// 调用方（插补进程）每周期按值构造传入 Step()
struct FsmContext {
    double norm_path_pos;   // 归一化路径位置 [0,1]
    double elapsed_time;    // action 启动起算秒数
    double joints[kFsmMaxJoints];
    double cart_pos[3];     // TCP 笛卡尔位置
    uint64_t cycle;         // 插补周期计数
};

// ============================================================================
// POD 静态断言
// ============================================================================
static_assert(std::is_standard_layout<FsmSignal>::value &&
              std::is_trivially_copyable<FsmSignal>::value, "FsmSignal must be POD");
static_assert(std::is_standard_layout<FsmTrigger>::value &&
              std::is_trivially_copyable<FsmTrigger>::value, "FsmTrigger must be POD");
static_assert(std::is_standard_layout<FsmFlag>::value &&
              std::is_trivially_copyable<FsmFlag>::value, "FsmFlag must be POD");
static_assert(std::is_standard_layout<FsmState>::value &&
              std::is_trivially_copyable<FsmState>::value, "FsmState must be POD");
static_assert(std::is_standard_layout<FsmTransition>::value &&
              std::is_trivially_copyable<FsmTransition>::value, "FsmTransition must be POD");
static_assert(std::is_standard_layout<FsmContext>::value &&
              std::is_trivially_copyable<FsmContext>::value, "FsmContext must be POD");
static_assert((kFsmSignalQueueDepth & (kFsmSignalQueueDepth - 1)) == 0,
              "kFsmSignalQueueDepth must be a power of 2");

#endif // __ACTION_FSM_TYPES_H__
