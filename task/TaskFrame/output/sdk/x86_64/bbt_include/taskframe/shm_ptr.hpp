#pragma once
#include <cstddef>
#include <cassert>
#include <utility>

extern void* g_shm_base;
extern size_t g_shm_size;

template<typename T>
class ShmPtr {
private:
    size_t _offset;

    T* ptr() const {
        assert(g_shm_base != nullptr);
        assert(_offset + sizeof(T) <= g_shm_size);
        return reinterpret_cast<T*>(static_cast<char*>(g_shm_base) + _offset);
    }

public:
    ShmPtr() : _offset(0) {}

    explicit ShmPtr(T* raw_ptr) : _offset(0) {
        if (raw_ptr != nullptr) {
            _offset = static_cast<size_t>(reinterpret_cast<char*>(raw_ptr) - static_cast<char*>(g_shm_base));
        }
    }

    explicit ShmPtr(size_t offset) : _offset(offset) {}

    size_t offset() const {
        return _offset;
    }

    T* get() const {
        if (_offset == 0) {
            return nullptr;
        }
        return ptr();
    }

    T* operator->() const {
        return get();
    }

    T& operator*() const {
        return *get();
    }

    explicit operator bool() const {
        return _offset != 0;
    }

    bool operator==(const ShmPtr<T>& other) const {
        return _offset == other._offset;
    }

    bool operator!=(const ShmPtr<T>& other) const {
        return _offset != other._offset;
    }

    bool operator==(std::nullptr_t) const {
        return _offset == 0;
    }

    bool operator!=(std::nullptr_t) const {
        return _offset != 0;
    }

    template<typename... Args>
    T* placement_new(Args&&... args) {
        return new (ptr()) T(std::forward<Args>(args)...);
    }
};
