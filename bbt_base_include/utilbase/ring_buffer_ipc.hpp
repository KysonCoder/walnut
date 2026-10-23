/**
 * @file ring_buffer_ipc.hpp
 * @brief Lock-free, single-writer/single-reader ring buffer for shared memory IPC.
 */

#ifndef BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_HPP_
#define BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_HPP_

#include "ring_buffer_ipc_data.hpp"
#include "logger.h"

namespace util {

template <typename T, uint32_t kCapacity>
class RingBufferIpc {
 public:
  explicit RingBufferIpc(RingBufferIpcData<T, kCapacity>* data) : data_(data) {}

  bool Init() {
    if (data_ == nullptr) {
      Log_E("RingBufferIpc::Init: null data pointer");
      return false;
    }
    data_->write_index.store(0, std::memory_order_relaxed);
    data_->read_index.store(0, std::memory_order_relaxed);
    return true;
  }

  bool Attach() {
    if (data_ == nullptr) {
      Log_E("RingBufferIpc::Attach: null data pointer");
      return false;
    }
    return true;
  }

  bool TryPush(const T& item) {
    const uint32_t write_index = data_->write_index.load(std::memory_order_relaxed);
    const uint32_t read_index = data_->read_index.load(std::memory_order_acquire);

    if (write_index - read_index >= kCapacity) {
      return false;
    }

    data_->buffer[write_index % kCapacity] = item;
    data_->write_index.store(write_index + 1, std::memory_order_release);
    return true;
  }

  bool TryPop(T& item) {
    const uint32_t read_index = data_->read_index.load(std::memory_order_relaxed);
    const uint32_t write_index = data_->write_index.load(std::memory_order_acquire);

    if (write_index == read_index) {
      return false;
    }

    item = data_->buffer[read_index % kCapacity];
    data_->read_index.store(read_index + 1, std::memory_order_release);
    return true;
  }

  bool IsEmpty() const {
    return data_->write_index.load(std::memory_order_acquire) ==
           data_->read_index.load(std::memory_order_acquire);
  }

  bool IsFull() const {
    const uint32_t write_index = data_->write_index.load(std::memory_order_acquire);
    const uint32_t read_index = data_->read_index.load(std::memory_order_acquire);
    return (write_index - read_index) >= kCapacity;
  }

  uint32_t Size() const {
    const uint32_t write_index = data_->write_index.load(std::memory_order_acquire);
    const uint32_t read_index = data_->read_index.load(std::memory_order_acquire);
    return write_index - read_index;
  }

  uint32_t Capacity() const { return kCapacity; }

  uint32_t FreeSpace() const {
    const uint32_t write_index = data_->write_index.load(std::memory_order_acquire);
    const uint32_t read_index = data_->read_index.load(std::memory_order_acquire);
    const uint32_t used = write_index - read_index;
    return used >= kCapacity ? 0 : kCapacity - used;
  }

  void Clear() {
    data_->write_index.store(0, std::memory_order_seq_cst);
    data_->read_index.store(0, std::memory_order_seq_cst);
  }

 private:
  RingBufferIpcData<T, kCapacity>* data_;
};

}  // namespace util

#endif  // BASE_UTILBASE_INCLUDE_RING_BUFFER_IPC_HPP_
