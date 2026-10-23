#pragma once
#include <cstdint>
#include <cstddef>
#include "shm_header.hpp"

#define TF_REGION_MAGIC_BASE 0x54462D5245470000ULL

enum class TfShmRegionId : uint32_t {
    EVENT = 0, MSG_QUEUE = 1, FEEDBACK = 2, STATUS = 3, DATA_POOL = 4, COUNT = 5
};

struct TfShmRegionEntry {
    uint64_t offset;
    uint64_t size;
    uint64_t magic;
    uint32_t version;
    uint32_t _pad;
};

struct TfShmRegionTable {
    uint32_t count;
    uint32_t version;
    TfShmRegionEntry entries[5];
};

class TfShmRegionManager {
public:
    static constexpr size_t REGION_EVENT_SIZE      = 1ULL * 1024 * 1024;
    static constexpr size_t REGION_MSG_QUEUE_SIZE  = 2ULL * 1024 * 1024;
    static constexpr size_t REGION_FEEDBACK_SIZE   = 8ULL * 1024 * 1024;
    static constexpr size_t REGION_STATUS_SIZE     = 64ULL * 1024;

    static size_t GetRegionSize(TfShmRegionId id);
    static size_t ComputeTotalRegionsSize();

    TfShmRegionManager();
    bool Init(void* shm_base, size_t shm_size, size_t region_table_offset);
    bool Attach(void* shm_base, size_t shm_size);
    void* GetRegionBase(TfShmRegionId id) const;
    size_t GetRegionSizeFromTable(TfShmRegionId id) const;
    bool IsRegionValid(TfShmRegionId id) const;

private:
    void* _shm_base;
    size_t _shm_size;
    TfShmRegionTable* _table;
    static uint64_t RegionMagic(TfShmRegionId id);
};
