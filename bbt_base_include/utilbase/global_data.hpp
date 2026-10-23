///@brief Global data pool template for shared memory object management
///
///@author Yonglin
///@version 0.1
///@date 05-may-2026

#ifndef _GLOBAL_DATA_H_
#define _GLOBAL_DATA_H_

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstring>
#include <cstdio>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "os.h"

template <typename T, unsigned int N>
class GlobalData
{
    static_assert(N > 0, "GlobalData pool size N must be greater than 0");
    static_assert(std::is_pod<T>::value, "GlobalData type T must be a POD type");

public:
    ///@brief 构造函数
    GlobalData();
    ///@brief 析构函数，自动断开连接
    ~GlobalData();

    ///@brief 创建共享内存池并初始化
    ///@param name 共享内存名称标识
    ///@param key  共享内存键值
    ///@return true 成功，false 失败
    bool Create(const char* name, int key);

    ///@brief 连接到已存在的共享内存池
    ///@param name 共享内存名称标识
    ///@param key  共享内存键值
    ///@return true 成功，false 失败
    bool Connect(const char* name, int key);

    ///@brief 从空闲池中获取一个对象
    ///@return 对象指针，池空时返回 nullptr
    T* Acquire();

    ///@brief 增加对象的引用计数
    ///@param obj 对象指针
    void Retain(T* obj);

    ///@brief 减少引用计数，若变为0则归还到空闲池
    ///@param obj 对象指针
    ///@return true 表示已归还到空闲池，false 表示引用仍大于0或对象无效
    bool PutBack(T* obj);

    ///@brief 获取对象的当前引用计数
    ///@param obj 对象指针
    ///@return 引用计数值，无效指针返回 -1
    int GetUseCount(const T* obj) const;

    ///@brief 获取当前使用中对象数
    ///@return 使用中对象数
    int UsedCount() const;

    ///@brief 获取空闲对象数
    ///@return 空闲对象数
    int AvailableCount() const;

    ///@brief 获取池总容量
    ///@return 总容量 N
    int Capacity() const;

    ///@brief 断开与共享内存的连接（不销毁共享内存）
    void Disconnect();

    ///@brief 销毁共享内存（仅创建者有效）
    void Destroy();

private:
    struct Node
    {
        T data;
        int32_t next_free;
        int use_count;
    };

    struct Pool
    {
        int32_t free_list_head;
        int initialized;
        int no_of_used_objects;
        int max_no_of_used_objects;
        Node nodes[N];
    };

    Pool* pool_;
    OS_MUTEX mutex_;
    char name_[64];
    int key_;
    bool is_creator_;
    int shm_fd_;

    int32_t ObjToIndex(const T* obj) const;
};

// ========== Implementation ==========

template <typename T, unsigned int N>
GlobalData<T, N>::GlobalData()
    : pool_(nullptr)
    , mutex_(nullptr)
    , key_(0)
    , is_creator_(false)
    , shm_fd_(-1)
{
    name_[0] = '\0';
}

template <typename T, unsigned int N>
GlobalData<T, N>::~GlobalData()
{
    Disconnect();
}

template <typename T, unsigned int N>
int32_t GlobalData<T, N>::ObjToIndex(const T* obj) const
{
    if (!obj || !pool_) return -1;
    const char* base = reinterpret_cast<const char*>(&pool_->nodes[0].data);
    const char* p = reinterpret_cast<const char*>(obj);
    ptrdiff_t offset = p - base;
    if (offset < 0 || offset % sizeof(Node) != 0) return -1;
    int32_t index = static_cast<int32_t>(offset / sizeof(Node));
    if (index < 0 || index >= static_cast<int32_t>(N)) return -1;
    return index;
}

template <typename T, unsigned int N>
int GlobalData<T, N>::Capacity() const
{
    return static_cast<int>(N);
}

template <typename T, unsigned int N>
int GlobalData<T, N>::UsedCount() const
{
    return pool_ ? pool_->no_of_used_objects : 0;
}

template <typename T, unsigned int N>
int GlobalData<T, N>::AvailableCount() const
{
    return pool_ ? (static_cast<int>(N) - pool_->no_of_used_objects) : 0;
}

template <typename T, unsigned int N>
int GlobalData<T, N>::GetUseCount(const T* obj) const
{
    int32_t index = ObjToIndex(obj);
    if (index < 0) return -1;
    return pool_->nodes[index].use_count;
}

template <typename T, unsigned int N>
bool GlobalData<T, N>::Create(const char* name, int key)
{
    if (!name || name[0] == '\0') return false;

    char shm_name[80];
    snprintf(shm_name, sizeof(shm_name), "/GlobalData_%s", name);

    int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return false;

    if (ftruncate(fd, sizeof(Pool)) < 0) {
        close(fd);
        return false;
    }

    void* addr = mmap(nullptr, sizeof(Pool), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        close(fd);
        return false;
    }

    pool_ = static_cast<Pool*>(addr);
    shm_fd_ = fd;

    mutex_ = os_mutex_create(const_cast<char*>(name));

    os_mutex_wait(mutex_, -1);
    if (pool_->initialized == 0) {
        memset(pool_, 0, sizeof(Pool));
        for (int i = static_cast<int>(N) - 1; i >= 0; i--) {
            pool_->nodes[i].next_free = (i > 0) ? (i - 1) : -1;
            pool_->nodes[i].use_count = -1;
        }
        pool_->free_list_head = static_cast<int32_t>(N - 1);
        pool_->no_of_used_objects = 0;
        pool_->max_no_of_used_objects = 0;
        pool_->initialized = 1;
        is_creator_ = true;
    }
    os_mutex_post(mutex_);

    strncpy(name_, name, sizeof(name_) - 1);
    name_[sizeof(name_) - 1] = '\0';
    key_ = key;
    return true;
}

template <typename T, unsigned int N>
bool GlobalData<T, N>::Connect(const char* name, int key)
{
    if (!name || name[0] == '\0') return false;

    char shm_name[80];
    snprintf(shm_name, sizeof(shm_name), "/GlobalData_%s", name);

    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd < 0) return false;

    void* addr = mmap(nullptr, sizeof(Pool), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        close(fd);
        return false;
    }

    pool_ = static_cast<Pool*>(addr);
    shm_fd_ = fd;

    mutex_ = os_mutex_create(const_cast<char*>(name));

    strncpy(name_, name, sizeof(name_) - 1);
    name_[sizeof(name_) - 1] = '\0';
    key_ = key;
    return true;
}

template <typename T, unsigned int N>
void GlobalData<T, N>::Disconnect()
{
    if (pool_ && pool_ != MAP_FAILED) {
        munmap(pool_, sizeof(Pool));
    }
    if (shm_fd_ >= 0) {
        close(shm_fd_);
    }
    pool_ = nullptr;
    shm_fd_ = -1;
    mutex_ = nullptr;
    name_[0] = '\0';
    key_ = 0;
    is_creator_ = false;
}

template <typename T, unsigned int N>
void GlobalData<T, N>::Destroy()
{
    if (!is_creator_) return;

    char shm_name[80];
    snprintf(shm_name, sizeof(shm_name), "/GlobalData_%s", name_);
    shm_unlink(shm_name);

    Disconnect();
}

template <typename T, unsigned int N>
T* GlobalData<T, N>::Acquire()
{
    if (!pool_) return nullptr;

    os_mutex_wait(mutex_, -1);

    if (pool_->free_list_head == -1) {
        os_mutex_post(mutex_);
        return nullptr;
    }

    int32_t index = pool_->free_list_head;
    Node& node = pool_->nodes[index];

    pool_->free_list_head = node.next_free;
    node.next_free = -1;
    node.use_count = 1;

    pool_->no_of_used_objects++;
    if (pool_->no_of_used_objects > pool_->max_no_of_used_objects)
        pool_->max_no_of_used_objects = pool_->no_of_used_objects;

    os_mutex_post(mutex_);
    return &node.data;
}

template <typename T, unsigned int N>
void GlobalData<T, N>::Retain(T* obj)
{
    if (!obj || !pool_) return;

    int32_t index = ObjToIndex(obj);
    if (index < 0 || index >= static_cast<int32_t>(N)) return;

    os_mutex_wait(mutex_, -1);
    if (pool_->nodes[index].next_free == -1) {
        pool_->nodes[index].use_count++;
    }
    os_mutex_post(mutex_);
}

template <typename T, unsigned int N>
bool GlobalData<T, N>::PutBack(T* obj)
{
    if (!obj || !pool_) return false;

    int32_t index = ObjToIndex(obj);
    if (index < 0 || index >= static_cast<int32_t>(N)) return false;

    os_mutex_wait(mutex_, -1);

    Node& node = pool_->nodes[index];
    if (node.next_free != -1) {
        os_mutex_post(mutex_);
        return false;
    }

    node.use_count--;
    if (node.use_count > 0) {
        os_mutex_post(mutex_);
        return false;
    }

    node.next_free = pool_->free_list_head;
    pool_->free_list_head = index;
    node.use_count = -1;
    pool_->no_of_used_objects--;

    os_mutex_post(mutex_);
    return true;
}

#endif // _GLOBAL_DATA_H_
