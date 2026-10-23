#include "tf_feedback.hpp"
#include <new>

TfFeedbackManager::TfFeedbackManager()
    : _region(nullptr)
    , _local_joints_valid(false)
    , _local_pose_valid(false)
    , _local_force_valid(false)
{
}

bool TfFeedbackManager::Init(void* region_base, size_t region_size) {
    if (region_base == nullptr) return false;
    if (region_size < sizeof(FeedbackRegion)) return false;

    _region = static_cast<FeedbackRegion*>(region_base);

    // Set header
    _region->magic = TF_FEEDBACK_REGION_MAGIC;
    _region->version = 1;
    _region->_pad = 0;

    // Placement-new each TripleBuffer to zero-initialize atomics
    new (&_region->joints) TripleBuffer<FEEDBACK_ALL_JOINTS>();
    new (&_region->pose)   TripleBuffer<FEEDBACK_ALL_POSE>();
    new (&_region->force)  TripleBuffer<FEEDBACK_ALL_FORCE>();

    // Zero the buffer arrays with memset
    std::memset(_region->joints.buffers, 0, sizeof(_region->joints.buffers));
    std::memset(_region->pose.buffers,   0, sizeof(_region->pose.buffers));
    std::memset(_region->force.buffers,  0, sizeof(_region->force.buffers));

    // Set initialized = true (release semantics so readers see all prior writes)
    _region->joints.initialized.store(true, std::memory_order_release);
    _region->pose.initialized.store(true, std::memory_order_release);
    _region->force.initialized.store(true, std::memory_order_release);

    return true;
}

bool TfFeedbackManager::Attach(void* region_base, size_t region_size) {
    if (region_base == nullptr) return false;
    if (region_size < sizeof(FeedbackRegion)) return false;

    FeedbackRegion* reg = static_cast<FeedbackRegion*>(region_base);
    if (reg->magic != TF_FEEDBACK_REGION_MAGIC) return false;

    _region = reg;
    return true;
}

// --- Writer algorithm ---
template<typename T>
void TfFeedbackManager::WriteTriple(TripleBuffer<T>* tb, const T& data) {
    uint32_t wi = tb->write_idx.load(std::memory_order_relaxed);
    // Mark "writing" (seq -> odd)
    uint64_t s = tb->seq.fetch_add(1, std::memory_order_acq_rel);
    // Write data
    tb->buffers[wi] = data;
    // Publish: update display_idx, then seq (even again)
    tb->display_idx.store(wi, std::memory_order_release);
    tb->seq.store(s + 2, std::memory_order_release);
    // Advance write_idx (skip display_idx)
    uint32_t next = (wi + 1) % 3;
    uint32_t di = tb->display_idx.load(std::memory_order_relaxed);
    if (next == di) {
        next = (next + 1) % 3;
    }
    tb->write_idx.store(next, std::memory_order_relaxed);
}

// --- Reader algorithm ---
template<typename T>
bool TfFeedbackManager::ReadTriple(TripleBuffer<T>* tb, T& out, T& local_cache, bool& local_valid) {
    if (!tb->initialized.load(std::memory_order_acquire)) {
        if (local_valid) { out = local_cache; return true; }
        return false;
    }
    const int MAX_RETRIES = 16;
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        uint64_t s1 = tb->seq.load(std::memory_order_acquire);
        if (s1 & 1ULL) continue;  // Writer active
        uint32_t di = tb->display_idx.load(std::memory_order_acquire);
        out = tb->buffers[di];
        std::atomic_thread_fence(std::memory_order_seq_cst);
        uint64_t s2 = tb->seq.load(std::memory_order_acquire);
        if (s1 == s2) {
            local_cache = out;
            local_valid = true;
            return true;
        }
    }
    // Max retries exceeded: use local cache
    if (local_valid) {
        out = local_cache;
        return true;
    }
    // No cache yet: one last attempt without retry limit
    uint64_t s1;
    do {
        s1 = tb->seq.load(std::memory_order_acquire);
        if (s1 & 1ULL) continue;
        uint32_t di = tb->display_idx.load(std::memory_order_acquire);
        out = tb->buffers[di];
        std::atomic_thread_fence(std::memory_order_seq_cst);
    } while (s1 != tb->seq.load(std::memory_order_acquire));
    local_cache = out;
    local_valid = true;
    return true;
}

// --- Explicit instantiations ---
template void TfFeedbackManager::WriteTriple(TripleBuffer<FEEDBACK_ALL_JOINTS>* tb, const FEEDBACK_ALL_JOINTS& data);
template void TfFeedbackManager::WriteTriple(TripleBuffer<FEEDBACK_ALL_POSE>* tb,   const FEEDBACK_ALL_POSE& data);
template void TfFeedbackManager::WriteTriple(TripleBuffer<FEEDBACK_ALL_FORCE>* tb,  const FEEDBACK_ALL_FORCE& data);

template bool TfFeedbackManager::ReadTriple(TripleBuffer<FEEDBACK_ALL_JOINTS>* tb, FEEDBACK_ALL_JOINTS& out, FEEDBACK_ALL_JOINTS& local_cache, bool& local_valid);
template bool TfFeedbackManager::ReadTriple(TripleBuffer<FEEDBACK_ALL_POSE>* tb,   FEEDBACK_ALL_POSE& out,   FEEDBACK_ALL_POSE& local_cache,   bool& local_valid);
template bool TfFeedbackManager::ReadTriple(TripleBuffer<FEEDBACK_ALL_FORCE>* tb,  FEEDBACK_ALL_FORCE& out,  FEEDBACK_ALL_FORCE& local_cache,  bool& local_valid);

// --- Public Update/Get methods ---
void TfFeedbackManager::UpdateJoints(const FEEDBACK_ALL_JOINTS& data) {
    if (_region) WriteTriple(&_region->joints, data);
}

bool TfFeedbackManager::GetJoints(FEEDBACK_ALL_JOINTS& out) {
    if (!_region) return false;
    return ReadTriple(&_region->joints, out, _local_joints, _local_joints_valid);
}

void TfFeedbackManager::UpdatePose(const FEEDBACK_ALL_POSE& data) {
    if (_region) WriteTriple(&_region->pose, data);
}

bool TfFeedbackManager::GetPose(FEEDBACK_ALL_POSE& out) {
    if (!_region) return false;
    return ReadTriple(&_region->pose, out, _local_pose, _local_pose_valid);
}

void TfFeedbackManager::UpdateForce(const FEEDBACK_ALL_FORCE& data) {
    if (_region) WriteTriple(&_region->force, data);
}

bool TfFeedbackManager::GetForce(FEEDBACK_ALL_FORCE& out) {
    if (!_region) return false;
    return ReadTriple(&_region->force, out, _local_force, _local_force_valid);
}
