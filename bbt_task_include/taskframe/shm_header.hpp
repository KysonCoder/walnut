#pragma once
#include <cstdint>

#define SHM_MAGIC 0x54462D53484D0000ULL  // "TF-SHM"

// SHM 布局版本。v2 = 2026-08-21 func/action 的 ActionData ABI 变更
// （sizeof/字段偏移改变，见 lib_docs/action_README.md），嵌入 shm 的
// ActionGroupSharedData 布局随之改变，旧布局必须被拒绝并重建。
constexpr uint32_t kShmLayoutVersion = 2;

struct ShmHeader {
    uint64_t magic = SHM_MAGIC;
    uint32_t version = kShmLayoutVersion;
    uint32_t size = 0;
    uint32_t crc = 0;
    uint32_t region_table_offset = 0;
};
