// 实时热路径验证（项目规则 4）：1 kHz × 5 s 事件发布/消费 + 反馈读写，
// 断言无数据损坏、seqnum 连续不丢事件、单帧延迟无毛刺。
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <sys/mman.h>

#include "tf_event.hpp"
#include "tf_feedback.hpp"
#include "tf_shm_region.hpp"

TEST(TfHotpathRt, EventAndFeedback1kHz) {
    void* mem = mmap(nullptr, 100 * 1024 * 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    ASSERT_NE(mem, MAP_FAILED);
    TfShmRegionManager region_mgr;
    ASSERT_TRUE(region_mgr.Init(mem, 100 * 1024 * 1024, sizeof(ShmHeader)));

    TfEventManager evt;
    ASSERT_TRUE(evt.Init(region_mgr.GetRegionBase(TfShmRegionId::EVENT),
                         region_mgr.GetRegionSizeFromTable(TfShmRegionId::EVENT), 2));
    TfFeedbackManager fb;
    ASSERT_TRUE(fb.Init(region_mgr.GetRegionBase(TfShmRegionId::FEEDBACK),
                        region_mgr.GetRegionSizeFromTable(TfShmRegionId::FEEDBACK)));

    constexpr int kCycles = 5000;  // 1 kHz × 5 s
    uint32_t consumed = 0;
    int64_t max_latency_us = 0;
    FEEDBACK_ALL_JOINTS joints = {};

    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < kCycles; ++i) {
        auto frame_start = std::chrono::steady_clock::now();

        // 事件：发布一条并立即消费（环深度 1，永不堆积）
        ASSERT_TRUE(evt.PublishEvent(0, EventType::CONTROL_ON)) << "cycle " << i;
        Event ev;
        ASSERT_TRUE(evt.ConsumeEvent(0, ev)) << "cycle " << i;
        EXPECT_EQ(ev.seqnum, consumed + 1);  // seqnum 连续 = 不丢事件
        consumed = ev.seqnum;

        // 反馈：写一帧读一帧，校验字段一致
        joints.joints[0].position_feedback = static_cast<double>(i);
        fb.UpdateJoints(joints);
        FEEDBACK_ALL_JOINTS out;
        ASSERT_TRUE(fb.GetJoints(out));
        EXPECT_DOUBLE_EQ(out.joints[0].position_feedback, static_cast<double>(i));

        auto frame_end = std::chrono::steady_clock::now();
        int64_t latency = std::chrono::duration_cast<std::chrono::microseconds>(
                              frame_end - frame_start).count();
        if (latency > max_latency_us) max_latency_us = latency;

        next += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next);
    }

    EXPECT_EQ(consumed, static_cast<uint32_t>(kCycles));
    // CI 宽松阈值：单帧热路径 < 5 ms（1 kHz 周期的 5 倍余量）
    EXPECT_LT(max_latency_us, 5000) << "max frame latency " << max_latency_us << " us";

    munmap(mem, 100 * 1024 * 1024);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
