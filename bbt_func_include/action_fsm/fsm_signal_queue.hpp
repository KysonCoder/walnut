/**
 * @file fsm_signal_queue.hpp
 * @brief actionFSM SPSC 无锁信号队列（POD，跨进程共享内存安全）
 * @date 2026-08-27
 *
 * 单写者 Push / 单读者 Pop。索引为普通 uint32_t，经 GCC __atomic 内建
 * 访问（acquire/release 序），x86_64 跨进程共享内存安全。
 * 队列满时 Push 返回 false——丢弃新信号，由调用方计 overflow_cnt
 * （最旧信号可能是 STOP，语义上更不可丢）。
 */
#ifndef __FSM_SIGNAL_QUEUE_H__
#define __FSM_SIGNAL_QUEUE_H__

#include "action_fsm_types.hpp"

struct FsmSignalQueue {
    uint32_t write_idx;   // 单调递增，取模寻址
    uint32_t read_idx;
    FsmSignal slots[kFsmSignalQueueDepth];

    void Reset() {
        __atomic_store_n(&write_idx, 0u, __ATOMIC_RELAXED);
        __atomic_store_n(&read_idx, 0u, __ATOMIC_RELAXED);
    }

    // 可读条数（任一方可调用，结果为瞬时值）
    unsigned int ReadLength() const {
        uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);
        uint32_t r = __atomic_load_n(&read_idx, __ATOMIC_RELAXED);
        return w - r;
    }

    // 单写者。满返回 false（丢弃新信号）。
    bool Push(const FsmSignal& sig) {
        constexpr uint32_t kMask = kFsmSignalQueueDepth - 1;
        uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_RELAXED);
        uint32_t r = __atomic_load_n(&read_idx, __ATOMIC_ACQUIRE);
        if (w - r >= (uint32_t)kFsmSignalQueueDepth) return false;
        slots[w & kMask] = sig;
        __atomic_store_n(&write_idx, w + 1, __ATOMIC_RELEASE);
        return true;
    }

    // 单读者。空返回 false。
    bool Pop(FsmSignal* out) {
        constexpr uint32_t kMask = kFsmSignalQueueDepth - 1;
        uint32_t r = __atomic_load_n(&read_idx, __ATOMIC_RELAXED);
        uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);
        if (w == r) return false;
        *out = slots[r & kMask];
        __atomic_store_n(&read_idx, r + 1, __ATOMIC_RELEASE);
        return true;
    }
};

static_assert(std::is_standard_layout<FsmSignalQueue>::value &&
              std::is_trivially_copyable<FsmSignalQueue>::value,
              "FsmSignalQueue must be POD");

#endif // __FSM_SIGNAL_QUEUE_H__
