#include <gtest/gtest.h>
#include "tf_shm_region.hpp"
#include "shm_header.hpp"
#include <sys/mman.h>

TEST(TfShmRegionTest, RegionLayoutFitsIn100MB) {
    size_t total = sizeof(ShmHeader) + TfShmRegionManager::ComputeTotalRegionsSize();
    EXPECT_LE(total, 100ULL * 1024 * 1024);
}

TEST(TfShmRegionTest, InitThenAttach) {
    void* mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    ASSERT_NE(mem, MAP_FAILED);
    ShmHeader* hdr = static_cast<ShmHeader*>(mem);
    hdr->magic = SHM_MAGIC; hdr->version = kShmLayoutVersion;

    TfShmRegionManager mgr;
    EXPECT_TRUE(mgr.Init(mem, 100 * 1024 * 1024, sizeof(ShmHeader)));
    EXPECT_GT(hdr->region_table_offset, 0u);

    for (int i = 0; i < 5; i++) {
        TfShmRegionId rid = static_cast<TfShmRegionId>(i);
        EXPECT_TRUE(mgr.IsRegionValid(rid));
        EXPECT_NE(mgr.GetRegionBase(rid), nullptr);
    }

    TfShmRegionManager mgr2;
    EXPECT_TRUE(mgr2.Attach(mem, 100 * 1024 * 1024));
    EXPECT_EQ(mgr2.GetRegionBase(TfShmRegionId::EVENT),
              mgr.GetRegionBase(TfShmRegionId::EVENT));
    munmap(mem, 100 * 1024 * 1024);
}

TEST(TfShmRegionTest, RegionSizesAreCorrect) {
    EXPECT_EQ(TfShmRegionManager::GetRegionSize(TfShmRegionId::EVENT),      1ULL * 1024 * 1024);
    EXPECT_EQ(TfShmRegionManager::GetRegionSize(TfShmRegionId::MSG_QUEUE),  2ULL * 1024 * 1024);
    EXPECT_EQ(TfShmRegionManager::GetRegionSize(TfShmRegionId::FEEDBACK),   8ULL * 1024 * 1024);
    EXPECT_EQ(TfShmRegionManager::GetRegionSize(TfShmRegionId::STATUS),     64ULL * 1024);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
