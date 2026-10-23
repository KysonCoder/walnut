#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include "tf_types.hpp"  // For FEEDBACK_ALL_JOINTS, FEEDBACK_ALL_POSE, FEEDBACK_ALL_FORCE

#define TF_FEEDBACK_REGION_MAGIC 0x54462D4644420000ULL

template<typename T>
struct TripleBuffer {
    static_assert(std::atomic<uint64_t>::is_always_lock_free,
                  "seqlock counter must be lock-free for cross-process use");
    static_assert(std::atomic<uint32_t>::is_always_lock_free,
                  "index atomics must be lock-free for cross-process use");
    static_assert(std::is_trivially_copyable_v<T>,
                  "TripleBuffer payload must be shm-safe");

    alignas(64) std::atomic<uint64_t> seq{0};
    alignas(64) std::atomic<uint32_t> write_idx{0};
    alignas(64) std::atomic<uint32_t> display_idx{0};
    alignas(64) std::atomic<bool> initialized{false};
    T buffers[3];
};

struct FeedbackRegion {
    uint64_t magic;
    uint32_t version;
    uint32_t _pad;
    TripleBuffer<FEEDBACK_ALL_JOINTS> joints;
    TripleBuffer<FEEDBACK_ALL_POSE> pose;
    TripleBuffer<FEEDBACK_ALL_FORCE> force;
};

class TfFeedbackManager {
public:
    TfFeedbackManager();
    bool Init(void* region_base, size_t region_size);
    bool Attach(void* region_base, size_t region_size);

    void UpdateJoints(const FEEDBACK_ALL_JOINTS& data);
    bool GetJoints(FEEDBACK_ALL_JOINTS& out);

    void UpdatePose(const FEEDBACK_ALL_POSE& data);
    bool GetPose(FEEDBACK_ALL_POSE& out);

    void UpdateForce(const FEEDBACK_ALL_FORCE& data);
    bool GetForce(FEEDBACK_ALL_FORCE& out);

private:
    FeedbackRegion* _region;
    FEEDBACK_ALL_JOINTS _local_joints;
    FEEDBACK_ALL_POSE   _local_pose;
    FEEDBACK_ALL_FORCE  _local_force;
    bool _local_joints_valid;
    bool _local_pose_valid;
    bool _local_force_valid;

    template<typename T>
    void WriteTriple(TripleBuffer<T>* tb, const T& data);

    template<typename T>
    bool ReadTriple(TripleBuffer<T>* tb, T& out, T& local_cache, bool& local_valid);
};
