/**
 * @file action_fsm_data.hpp
 * @brief ActionFsmData —— actionFSM 全部可共享状态的 POD 镜像
 * @date 2026-08-27
 *
 * 整体放共享内存（utilbase GlobalData<ActionFsmData,N> 池）。
 * 单写者约定：
 *   注入方（外部进程）只写 signal_queue / urgent_* / overflow_cnt / urgent_lost_cnt
 *   步进方（插补进程）只写状态字段 / triggers[].fired / flags[] / invalid_transition_cnt
 *   交集字段（队列索引、urgent 序号、flag.set）经 __atomic 内建访问
 */
#ifndef __ACTION_FSM_DATA_H__
#define __ACTION_FSM_DATA_H__

#include "action_fsm_types.hpp"
#include "fsm_signal_queue.hpp"

struct ActionFsmData {
    // ---- 头部 ----
    uint32_t magic;              // kFsmMagic
    uint32_t version;            // kFsmVersion
    uint8_t  configured;         // LoadConfig 校验通过后置 1
    uint8_t  state_count;
    uint8_t  transition_count;
    uint8_t  trigger_count;

    // ---- 配置表（创建进程写，运行期只读）----
    FsmState      states[kFsmMaxStates];
    FsmTransition transitions[kFsmMaxTransitions];
    FsmTrigger    triggers[kFsmMaxTriggers];

    // ---- 运行态（步进方写）----
    uint8_t  current_state;
    uint8_t  prev_state;
    uint8_t  initial_state;
    uint8_t  motion_permitted;   // = states[current_state].permits_motion
    uint64_t state_enter_cycle;
    uint64_t cycle_count;
    double   speed_override;     // 1.0 = 100%，SET_OVERRIDE 更新，钳位 [0,2]

    // ---- 输入：信号队列（注入方写，步进方读）----
    FsmSignalQueue signal_queue;

    // ---- 输入：紧急槽位（写覆盖，Step 最先消费）----
    uint32_t urgent_seq;           // 最新紧急信号序号；__atomic
    uint32_t urgent_consumed_seq;  // 步进方已消费序号；__atomic；注入方据此检测覆盖丢失
    FsmSignal urgent_signal;

    // ---- 输出标志（步进方置位，外部进程轮询/ClearFlag）----
    FsmFlag flags[kFsmMaxFlags];

    // ---- 诊断计数 ----
    uint32_t overflow_cnt;            // 队列满丢弃（注入方写）
    uint32_t invalid_transition_cnt;  // 无匹配转移（步进方写）
    uint32_t urgent_lost_cnt;         // 紧急槽位覆盖丢失（注入方写）
    uint32_t reserved_diag;
};

static_assert(std::is_standard_layout<ActionFsmData>::value,
              "ActionFsmData must be standard-layout for shared memory");
static_assert(std::is_trivially_copyable<ActionFsmData>::value,
              "ActionFsmData must be trivially-copyable for shared memory");
static_assert(std::is_pod<ActionFsmData>::value,
              "ActionFsmData must be POD for GlobalData<T,N> pool");

// memset 0 + 头部/默认值/flag id 初始化。nullptr 安全。
void InitActionFsmData(ActionFsmData* data);

#endif // __ACTION_FSM_DATA_H__
