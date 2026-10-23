#include <gtest/gtest.h>
#include "shm_ptr.hpp"

TEST(ShmPtr, BasicOffset) {
    alignas(alignof(int)) char mem[1024];
    g_shm_base = mem;
    g_shm_size = sizeof(mem);

    int* raw = reinterpret_cast<int*>(&mem[128]);
    *raw = 42;

    ShmPtr<int> p(raw);
    EXPECT_EQ(p.offset(), 128u);
    EXPECT_EQ(p.get(), raw);
    EXPECT_EQ(*p, 42);
    EXPECT_EQ(p.operator->(), raw);
    EXPECT_TRUE(static_cast<bool>(p));

    // Reset globals for other tests
    g_shm_base = nullptr;
    g_shm_size = 0;
}

TEST(ShmPtr, NullPointer) {
    ShmPtr<int> p;
    EXPECT_EQ(p.offset(), 0u);
    EXPECT_EQ(p.get(), nullptr);
    EXPECT_FALSE(static_cast<bool>(p));
    EXPECT_TRUE(p == nullptr);
    EXPECT_FALSE(p != nullptr);
}

TEST(ShmPtr, PlacementNew) {
    alignas(alignof(int)) char mem[1024];
    g_shm_base = mem;
    g_shm_size = sizeof(mem);

    ShmPtr<int> p(64);  // offset 64
    int* constructed = p.placement_new(123);
    EXPECT_EQ(constructed, reinterpret_cast<int*>(&mem[64]));
    EXPECT_EQ(*constructed, 123);
    EXPECT_EQ(*p, 123);

    // Clean up (destructor call for int is a no-op, but good practice)
    using IntAlias = int;
    constructed->~IntAlias();

    g_shm_base = nullptr;
    g_shm_size = 0;
}
