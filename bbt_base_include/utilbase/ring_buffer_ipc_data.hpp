/**
 * @file ring_buffer_ipc_data.hpp
 * @brief Shared-memory-resident data struct for RingBufferIPC.
 */

#ifndef BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_DATA_HPP_
#define BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_DATA_HPP_

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace util {

template <typename T, uint32_t kCapacity>
struct RingBufferIpcData {
  static_assert(kCapacity > 0, "Capacity must be > 0");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "uint32_t atomic must be lock-free for cross-process use");
  static_assert(std::is_trivially_copyable_v<T>,
                "T must be trivially copyable");
  static_assert(std::is_standard_layout_v<T>,
                "T must be standard layout");
  static_assert(std::is_trivially_destructible_v<T>,
                "T must be trivially destructible for shared memory safety");

  std::atomic<uint32_t> write_index{0};
  std::atomic<uint32_t> read_index{0};
  T buffer[kCapacity];
};

}  // namespace util

#endif  // BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_DATA_HPP_
