#include "tf_shm_region.hpp"
#include <cstring>

size_t TfShmRegionManager::GetRegionSize(TfShmRegionId id) {
    switch (id) {
        case TfShmRegionId::EVENT:      return REGION_EVENT_SIZE;
        case TfShmRegionId::MSG_QUEUE:  return REGION_MSG_QUEUE_SIZE;
        case TfShmRegionId::FEEDBACK:   return REGION_FEEDBACK_SIZE;
        case TfShmRegionId::STATUS:     return REGION_STATUS_SIZE;
        case TfShmRegionId::DATA_POOL:  return 0; // computed dynamically
        default:                        return 0;
    }
}

size_t TfShmRegionManager::ComputeTotalRegionsSize() {
    return REGION_EVENT_SIZE + REGION_MSG_QUEUE_SIZE + REGION_FEEDBACK_SIZE + REGION_STATUS_SIZE;
}

uint64_t TfShmRegionManager::RegionMagic(TfShmRegionId id) {
    return TF_REGION_MAGIC_BASE + static_cast<uint32_t>(id);
}

TfShmRegionManager::TfShmRegionManager()
    : _shm_base(nullptr), _shm_size(0), _table(nullptr) {
}

bool TfShmRegionManager::Init(void* shm_base, size_t shm_size, size_t region_table_offset) {
    if (!shm_base || shm_size == 0) {
        return false;
    }

    _shm_base = shm_base;
    _shm_size = shm_size;
    _table = reinterpret_cast<TfShmRegionTable*>(
        static_cast<uint8_t*>(shm_base) + region_table_offset);

    // Set the region_table_offset in ShmHeader
    ShmHeader* hdr = static_cast<ShmHeader*>(shm_base);
    hdr->region_table_offset = static_cast<uint32_t>(region_table_offset);

    // Initialize the region table
    _table->count = 5;
    _table->version = 1;

    // Compute starting offset for the first region (after the header and region table)
    size_t current_offset = region_table_offset + sizeof(TfShmRegionTable);

    for (uint32_t i = 0; i < 5; i++) {
        TfShmRegionId rid = static_cast<TfShmRegionId>(i);
        size_t region_size = GetRegionSize(rid);

        // DATA_POOL size is computed dynamically as the remaining space
        if (rid == TfShmRegionId::DATA_POOL) {
            if (current_offset > shm_size) {
                return false;
            }
            region_size = shm_size - current_offset;
        }

        _table->entries[i].offset = current_offset;
        _table->entries[i].size = region_size;
        _table->entries[i].magic = RegionMagic(rid);
        _table->entries[i].version = 1;
        _table->entries[i]._pad = 0;

        current_offset += region_size;
    }

    // Overflow check: total must not exceed shm_size
    if (current_offset > shm_size) {
        return false;
    }

    return true;
}

bool TfShmRegionManager::Attach(void* shm_base, size_t shm_size) {
    if (!shm_base || shm_size == 0) {
        return false;
    }

    ShmHeader* hdr = static_cast<ShmHeader*>(shm_base);
    if (hdr->magic != SHM_MAGIC || hdr->region_table_offset == 0) {
        return false;
    }

    _shm_base = shm_base;
    _shm_size = shm_size;
    _table = reinterpret_cast<TfShmRegionTable*>(
        static_cast<uint8_t*>(shm_base) + hdr->region_table_offset);

    // Validate table count
    if (_table->count != static_cast<uint32_t>(TfShmRegionId::COUNT)) {
        return false;
    }

    // Validate magic on each entry and bounds
    for (uint32_t i = 0; i < 5; i++) {
        TfShmRegionId rid = static_cast<TfShmRegionId>(i);
        if (_table->entries[i].magic != RegionMagic(rid)) {
            return false;
        }
        if (_table->entries[i].offset + _table->entries[i].size > shm_size) {
            return false;
        }
    }

    return true;
}

void* TfShmRegionManager::GetRegionBase(TfShmRegionId id) const {
    uint32_t idx = static_cast<uint32_t>(id);
    if (idx >= 5 || !_table) {
        return nullptr;
    }
    return static_cast<uint8_t*>(_shm_base) + _table->entries[idx].offset;
}

size_t TfShmRegionManager::GetRegionSizeFromTable(TfShmRegionId id) const {
    uint32_t idx = static_cast<uint32_t>(id);
    if (idx >= 5 || !_table) {
        return 0;
    }
    return _table->entries[idx].size;
}

bool TfShmRegionManager::IsRegionValid(TfShmRegionId id) const {
    uint32_t idx = static_cast<uint32_t>(id);
    if (idx >= 5 || !_table) {
        return false;
    }
    return _table->entries[idx].magic == RegionMagic(id);
}
