/**
 * @file planning_runner_test.cpp
 * @brief PlanningRunner 单元测试：真实 confbase 配置 + 本地轨迹环（无 shm）。
 */

#include <gtest/gtest.h>
#include <cstdlib>
#include <cstring>

#include "planning_runner.hpp"
#include "utilbase/ring_buffer_ipc_data.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr const char* kUrdfPath = "/home/user/claude/cfg/humanoid.urdf";
constexpr const char* kCfgPath = "/home/user/claude/cfg/humanoid_robot_conf.cfg";

// humanoid 右臂（humanoid_conf.hpp: HumanoidSocUnitType::RIGHT_ARM = 0；
// 2026-08-26 func probe 证实 num_joints=7，model CKinGeneRbt）
constexpr int kRightArmSoc = 0;
// humanoid 左臂（humanoid_conf.hpp: HumanoidSocUnitType::LEFT_ARM = 6，num_joints=7）
constexpr int kLeftArmSoc = 6;
constexpr int kArmJoints = 7;

// unit 的全局逻辑关节号列表（与 servo_engine_test 同款映射：
// global_logical = global_map->physical_to_logical[unit->logical_to_physical[i]]）
static std::vector<int> UnitLogicalJoints(robot_config::RobotSocUnitType unit_type) {
    auto& store = robot_config::RobotConfigStore::Instance();
    const robot_config::HumanoidUnitConfig* unit = store.GetUnit(unit_type);
    const robot_config::HumanoidGlobalJointMap* map = store.GetGlobalJointMap();
    std::vector<int> joints;
    if (unit == nullptr || map == nullptr) return joints;
    for (int i = 0; i < unit->num_joints; i++) {
        int32_t phys = unit->logical_to_physical[i];
        if (phys < 0) continue;
        int32_t logical = map->physical_to_logical[phys];
        if (logical >= 0 && logical < static_cast<int32_t>(robot_config::kRobotMaxJoints))
            joints.push_back(logical);
    }
    return joints;
}

class PlanningRunnerTest : public ::testing::Test {
protected:
    void* config_mem_ = nullptr;
    size_t config_size_ = 0;
    // typed store 为单例（HumanoidConfigStore 私有构造），进程内共享。
    robot_config::RobotConfigStore& store_ = robot_config::RobotConfigStore::Instance();
    util::RingBufferIpcData<spline::PathSplineData, 64> ring_data_{};
    util::RingBufferIpc<spline::PathSplineData, 64> ring_{&ring_data_};
    PlanningRunner runner_;

    void SetUp() override {
        // 真实配置：builder 序列化到内存，store 直接 attach（无 shm 依赖）。
        robot_config::RobotConfigBuilder builder;
        ASSERT_TRUE(builder.Init());
        ASSERT_TRUE(builder.LoadDriveParams(kCfgPath));
        ASSERT_TRUE(builder.LoadUrdfKinematics(kUrdfPath));
        config_size_ = builder.GetSerializedSize();
        ASSERT_GT(config_size_, 0u);
        ASSERT_EQ(posix_memalign(&config_mem_, 4096, config_size_), 0);
        size_t written = 0;
        ASSERT_TRUE(builder.BuildToMemory(config_mem_, config_size_, &written));
        ASSERT_EQ(written, config_size_);
        store_.Detach();
        ASSERT_EQ(store_.Attach(config_mem_, config_size_), robot_config::ConfigError::OK);
        ring_.Init();
    }

    void TearDown() override {
        store_.Detach();
        free(config_mem_);
        config_mem_ = nullptr;
    }

    // 起始姿态：README 示例的就绪姿态（零位姿 FK 退化，IK 不可解）。
    static constexpr double kReadyPose[kArmJoints] = {0, -0.5, 0.5, 0, 0.5, 0, 0};

    // 合成单段关节动作组：右臂 7 关节在就绪姿态上小角度偏移（关节目标，避开 IK 可达性）。
    static ActionGroupSharedData MakeJointActionGroup() {
        ActionGroupSharedData group{};
        ActionData& a = group.actions[kRightArmSoc];
        a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
        a.unit_type = static_cast<SOC_UNIT_TYPE>(kRightArmSoc);
        a.valid_start_target = false;
        TargetData& t = a.end_target;
        t.unit_type = a.unit_type;
        for (int j = 0; j < kArmJoints; ++j) t.joints[j] = kReadyPose[j] + 0.05 * (j + 1);
        t.joint_number = kArmJoints;
        t.is_joints_valid = true;
        group.soc_enable[kRightArmSoc] = true;
        return group;
    }

    // 反馈 = 就绪姿态（humanoid 右臂局部关节即全局前 7 个——若映射断言失败
    // 再按 GlobalJointMap 修正）。
    static FEEDBACK_ALL_JOINTS ReadyFeedback() {
        FEEDBACK_ALL_JOINTS fb{};
        for (int j = 0; j < kArmJoints; ++j) fb.joints[j].position_feedback = kReadyPose[j];
        return fb;
    }

    // 双臂关节动作组：左右臂各自 7 关节小角度偏移（产生 ≥2 段，覆盖段级暂停/续传）
    static ActionGroupSharedData MakeDualArmJointActionGroup() {
        ActionGroupSharedData group = MakeJointActionGroup();  // 右臂
        ActionData& a = group.actions[kLeftArmSoc];
        a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
        a.unit_type = static_cast<SOC_UNIT_TYPE>(kLeftArmSoc);
        a.valid_start_target = false;
        TargetData& t = a.end_target;
        t.unit_type = a.unit_type;
        for (int j = 0; j < kArmJoints; ++j) t.joints[j] = kReadyPose[j] + 0.05 * (j + 1);
        t.joint_number = kArmJoints;
        t.is_joints_valid = true;
        group.soc_enable[kLeftArmSoc] = true;
        return group;
    }

    // 双臂反馈：按全局映射把两臂关节都置为就绪姿态
    static FEEDBACK_ALL_JOINTS DualArmReadyFeedback() {
        FEEDBACK_ALL_JOINTS fb{};
        const std::vector<int> r =
            UnitLogicalJoints(robot_config::RobotSocUnitType::RIGHT_ARM);
        const std::vector<int> l =
            UnitLogicalJoints(robot_config::RobotSocUnitType::LEFT_ARM);
        for (int j = 0; j < kArmJoints; ++j) {
            fb.joints[r[j]].position_feedback = kReadyPose[j];
            fb.joints[l[j]].position_feedback = kReadyPose[j];
        }
        return fb;
    }
};

TEST_F(PlanningRunnerTest, InitSucceedsWithRealConfig) {
    EXPECT_TRUE(runner_.Init(store_, &ring_));
    EXPECT_TRUE(runner_.IsInitialized());
}

TEST_F(PlanningRunnerTest, InitRejectsNullTrajList) {
    EXPECT_FALSE(runner_.Init(store_, nullptr));
    EXPECT_FALSE(runner_.IsInitialized());
}

TEST_F(PlanningRunnerTest, ProcessSingleJointActionSucceeds) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    const ActionGroupSharedData group = MakeJointActionGroup();

    EXPECT_TRUE(runner_.ProcessActionGroup(group, ReadyFeedback()))
        << runner_.last_result().message;

    // 逐段弹出 PathSplineData（段数由管线决定，≥1），校验链式连续：
    // 首段 l=0 为就绪姿态，末段 l=1 为目标关节，相邻段端点一致。
    std::array<double, robot_config::kRobotMaxJoints> ones_data{};
    std::array<bool, robot_config::kRobotMaxJoints> all_true{};
    ones_data.fill(1.0);
    all_true.fill(true);

    int segment_count = 0;
    double total_time = 0.0;
    std::array<double, robot_config::kRobotMaxJoints> prev_end{};
    spline::PathSplineData out{};
    while (ring_.TryPop(out)) {
        ++segment_count;
        total_time += out.path_time;
        spline::PathSpline path(&out);
        EXPECT_GT(out.path_time, 0.0);
        double l = -1.0;
        ASSERT_TRUE(path.GetL(0.0, l));
        EXPECT_NEAR(l, 0.0, 1e-9);
        ASSERT_TRUE(path.GetL(out.path_time, l));
        EXPECT_NEAR(l, 1.0, 1e-9);

        spline::JointArray q_start(ones_data, all_true);  // 占位，Eval 覆写
        spline::JointArray q_end(ones_data, all_true);
        ASSERT_TRUE(path.joint_spline().Eval(0.0, q_start));
        ASSERT_TRUE(path.joint_spline().Eval(1.0, q_end));
        if (segment_count == 1) {
            for (int j = 0; j < kArmJoints; ++j)
                EXPECT_NEAR(q_start[j], kReadyPose[j], 1e-6);
        } else {
            for (int j = 0; j < robot_config::kRobotMaxJoints; ++j)
                EXPECT_NEAR(q_start[j], prev_end[j], 1e-9);
        }
        for (int j = 0; j < robot_config::kRobotMaxJoints; ++j)
            prev_end[j] = q_end[j];
    }
    EXPECT_GE(segment_count, 1);
    EXPECT_GT(total_time, 0.0);
    // 末段终点 = 目标关节。
    for (int j = 0; j < kArmJoints; ++j)
        EXPECT_NEAR(prev_end[j], kReadyPose[j] + 0.05 * (j + 1), 1e-6);
}

TEST_F(PlanningRunnerTest, ProcessUnreachableActionFails) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ActionGroupSharedData group{};
    ActionData& a = group.actions[kRightArmSoc];
    a.motion_type = ACTION_MOTION_TYPE::ACTION_LINE;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(kRightArmSoc);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    t.pose.pos.x = 100.0;  // 远超工作空间
    t.pose.pos.y = 100.0;
    t.pose.pos.z = 100.0;
    t.pose.orient.q0 = 1.0;
    t.is_position_valid = true;
    t.is_orientation_valid = true;
    t.is_joints_valid = false;
    group.soc_enable[kRightArmSoc] = true;

    EXPECT_FALSE(runner_.ProcessActionGroup(group, ReadyFeedback()));
    EXPECT_FALSE(runner_.last_result().ok);
    spline::PathSplineData out{};
    EXPECT_FALSE(ring_.TryPop(out));  // 队列无残留
}

TEST_F(PlanningRunnerTest, ProcessWithoutInitFails) {
    PlanningRunner fresh;
    EXPECT_FALSE(fresh.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()));
}

// 直构 kJoint 段：关节空间线性插值（端点精确、length_m 为 rad 口径）
TEST_F(PlanningRunnerTest, ConvertJointSegmentProducesExactJointLine) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));

    action::PathActionSegment seg{};
    seg.type = action::SegmentType::kJoint;
    double q0[kArmJoints], q1[kArmJoints];
    for (int j = 0; j < kArmJoints; ++j) {
        q0[j] = kReadyPose[j];
        q1[j] = kReadyPose[j] + 0.05 * (j + 1);
        seg.start_target_data.joints[j] = q0[j];
        seg.end_target_data.joints[j] = q1[j];
    }
    double len_sq = 0.0;
    for (int j = 0; j < kArmJoints; ++j) len_sq += (q1[j] - q0[j]) * (q1[j] - q0[j]);
    seg.length_m = std::sqrt(len_sq);   // kJoint: 关节路径长度 [rad]
    seg.L_start = 0.0;
    seg.L_end = 1.0;
    seg.v_enter = 0.0;                  // fine point：库侧保证两端 v=0
    seg.v_exit = 0.0;
    seg.v_cruise = 0.5;
    seg.timestamp = 0.0;

    spline::PathSplineData out{};
    ASSERT_TRUE(runner_.ConvertSegment(kRightArmSoc, seg, 1.0, &out));

    spline::PathSpline path(&out);
    EXPECT_NEAR(out.main_length, seg.length_m, 1e-12);
    std::array<double, robot_config::kRobotMaxJoints> ones{};
    ones.fill(1.0);
    std::array<bool, robot_config::kRobotMaxJoints> all_true{};
    all_true.fill(true);
    spline::JointArray qa(ones, all_true), qb(ones, all_true);
    ASSERT_TRUE(path.joint_spline().Eval(0.0, qa));
    ASSERT_TRUE(path.joint_spline().Eval(1.0, qb));
    for (int j = 0; j < kArmJoints; ++j) {
        EXPECT_NEAR(qa[j], q0[j], 1e-9);
        EXPECT_NEAR(qb[j], q1[j], 1e-9);
    }
}

// func 2026-09-02 R1 端到端：ACTION_JOINT 动作组经 RunAll 产出 kJoint 段、
// fine point（run 两端 v_L=0）、末段终点关节精确等于目标
TEST_F(PlanningRunnerTest, ProcessJointActionProducesJointSegments) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()))
        << runner_.last_result().message;

    const action::PathActionGroup& result = runner_.last_group();
    const action::PathActionSequence& seq = result.arms[kRightArmSoc];
    ASSERT_FALSE(seq.segments.empty());
    for (const auto& seg : seq.segments)
        EXPECT_EQ(seg.type, action::SegmentType::kJoint);
    EXPECT_DOUBLE_EQ(seq.segments.front().v_enter, 0.0);  // fine point 入口
    EXPECT_DOUBLE_EQ(seq.segments.back().v_exit, 0.0);    // fine point 出口
    for (int j = 0; j < kArmJoints; ++j)
        EXPECT_NEAR(seq.segments.back().end_target_data.joints[j],
                    kReadyPose[j] + 0.05 * (j + 1), 1e-9);
}

// enabled SOC 携带 ACTION_NULL（零初始化）→ task 侧入口拒绝（func 语义是
// "该 SOC 不动"，放行即静默丢失动作）
TEST_F(PlanningRunnerTest, ProcessActionGroupRejectsNullMotionType) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ActionGroupSharedData group = MakeJointActionGroup();
    group.actions[kRightArmSoc].motion_type = ACTION_MOTION_TYPE::ACTION_NULL;
    EXPECT_FALSE(runner_.ProcessActionGroup(group, ReadyFeedback()));
    spline::PathSplineData out{};
    EXPECT_FALSE(ring_.TryPop(out));  // 队列无残留
}

// enabled SOC 携带 func 不支持的类型 → task 侧入口拒绝（带 soc 定位，
// 不进 RunAll）
TEST_F(PlanningRunnerTest, ProcessActionGroupRejectsUnsupportedMotionType) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ActionGroupSharedData group = MakeJointActionGroup();
    group.actions[kRightArmSoc].motion_type = ACTION_MOTION_TYPE::ACTION_CIRCLE;
    EXPECT_FALSE(runner_.ProcessActionGroup(group, ReadyFeedback()));
    spline::PathSplineData out{};
    EXPECT_FALSE(ring_.TryPop(out));
}

// 未知段类型：防御性失败（前向兼容墙）
TEST_F(PlanningRunnerTest, ConvertUnknownSegmentTypeFails) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    action::PathActionSegment seg{};
    seg.type = static_cast<action::SegmentType>(99);
    seg.start_target_data.joints[0] = 0.0;
    seg.end_target_data.joints[0] = 0.1;
    seg.length_m = 0.1;
    seg.L_start = 0.0;
    seg.L_end = 1.0;
    seg.v_cruise = 0.5;

    spline::PathSplineData out{};
    EXPECT_FALSE(runner_.ConvertSegment(kRightArmSoc, seg, 1.0, &out));
}

TEST_F(PlanningRunnerTest, WaterLevelAccountingMatchesRingConsumption) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()))
        << runner_.last_result().message;

    // 逐段弹出（模拟 Servo 消费）：每弹一段水位精确下降该段 path_time
    //（累减 vs FIFO 累加存在浮点噪声，容差 1e-12）
    double level = runner_.BufferedTimeS();
    ASSERT_GT(level, 0.0);
    spline::PathSplineData out{};
    while (ring_.TryPop(out)) {
        level -= out.path_time;
        EXPECT_NEAR(runner_.BufferedTimeS(), level, 1e-12);
    }
    EXPECT_DOUBLE_EQ(runner_.BufferedTimeS(), 0.0);
}

TEST_F(PlanningRunnerTest, WaterLevelDesyncTreatsAsFull) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    // 环内残留段（模拟 Planning 重启前遗留），FIFO 为空 → 失账按水位已满处理
    spline::PathSplineData orphan{};
    orphan.path_time = 1.0;
    ASSERT_TRUE(ring_.TryPush(orphan));
    EXPECT_GT(runner_.BufferedTimeS(), PlanningRunner::kTrajPrefetchTimeS);
    // Servo 消费残留段后对账恢复
    spline::PathSplineData out{};
    ASSERT_TRUE(ring_.TryPop(out));
    EXPECT_DOUBLE_EQ(runner_.BufferedTimeS(), 0.0);
}

TEST_F(PlanningRunnerTest, GatedEmitPausesAtPrefetchAndResumesInOrder) {
    const ActionGroupSharedData group = MakeDualArmJointActionGroup();
    const FEEDBACK_ALL_JOINTS fb = DualArmReadyFeedback();

    // 无门控参考序列（独立 runner + 环，走兼容包装全量发射）
    util::RingBufferIpcData<spline::PathSplineData, 64> ref_data{};
    util::RingBufferIpc<spline::PathSplineData, 64> ref_ring{&ref_data};
    PlanningRunner ref_runner;
    ASSERT_TRUE(ref_runner.Init(store_, &ref_ring));
    ASSERT_TRUE(ref_runner.ProcessActionGroup(group, fb)) << ref_runner.last_result().message;
    std::vector<spline::PathSplineData> ref;
    spline::PathSplineData out{};
    double ref_total = 0.0;
    while (ref_ring.TryPop(out)) { ref.push_back(out); ref_total += out.path_time; }
    ASSERT_GE(ref.size(), 2u);  // 双臂组应产生 ≥2 段，否则本测试无意义
    ASSERT_GT(ref_total, PlanningRunner::kTrajPrefetchTimeS);  // 总时长必须超阈值

    // 门控发射：水位 ≥96ms 即暂停，且只发了一部分
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.PlanActionGroup(group, fb)) << runner_.last_result().message;
    EXPECT_EQ(runner_.EmitPending(), PlanningRunner::EmitResult::kPaused);
    EXPECT_TRUE(runner_.HasPending());
    EXPECT_GE(runner_.BufferedTimeS(), PlanningRunner::kTrajPrefetchTimeS);
    ASSERT_GE(ring_.Size(), 1u);
    ASSERT_LT(ring_.Size(), ref.size());

    // 模拟 Servo 排空：水位超出阈值的部分 < 最近推入的一段
    std::vector<spline::PathSplineData> got;
    double w = 0.0;
    spline::PathSplineData last{};
    while (ring_.TryPop(out)) { w += out.path_time; last = out; got.push_back(out); }
    EXPECT_LT(w - last.path_time, PlanningRunner::kTrajPrefetchTimeS);
    EXPECT_DOUBLE_EQ(runner_.BufferedTimeS(), 0.0);  // 对账清零

    // 续传完成（每轮至少推 1 段——排空后水位 0 < 阈值；轮数上限取环容量 2 倍，
    // 覆盖任意段数）
    for (int i = 0; i < 128 && runner_.EmitPending() != PlanningRunner::EmitResult::kDone; i++) {
        while (ring_.TryPop(out)) got.push_back(out);
    }
    while (ring_.TryPop(out)) got.push_back(out);
    EXPECT_FALSE(runner_.HasPending());

    // 与无门控参考逐段一致：门控只改变发射时机，不改变段内容与顺序
    ASSERT_EQ(got.size(), ref.size());
    for (size_t i = 0; i < got.size(); i++) {
        EXPECT_DOUBLE_EQ(got[i].path_time, ref[i].path_time) << "seg " << i;
        EXPECT_DOUBLE_EQ(got[i].main_length, ref[i].main_length) << "seg " << i;
        EXPECT_EQ(std::memcmp(&got[i].joint_spline_data, &ref[i].joint_spline_data,
                              sizeof(got[i].joint_spline_data)), 0) << "seg " << i;
    }
}

TEST_F(PlanningRunnerTest, AbortPendingDropsRemainingEmission) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.PlanActionGroup(MakeDualArmJointActionGroup(),
                                        DualArmReadyFeedback()))
        << runner_.last_result().message;
    EXPECT_EQ(runner_.EmitPending(), PlanningRunner::EmitResult::kPaused);
    ASSERT_TRUE(runner_.HasPending());

    runner_.AbortPending();
    EXPECT_FALSE(runner_.HasPending());

    // FIFO 已清但环内有残留 → 失账按水位已满；排空后对账恢复
    EXPECT_GT(runner_.BufferedTimeS(), PlanningRunner::kTrajPrefetchTimeS);
    spline::PathSplineData out{};
    while (ring_.TryPop(out)) {}
    EXPECT_DOUBLE_EQ(runner_.BufferedTimeS(), 0.0);

    // 可重新规划并发射新组（单臂组如仍超阈值则排空续传至发完）
    ASSERT_TRUE(runner_.PlanActionGroup(MakeJointActionGroup(), ReadyFeedback()))
        << runner_.last_result().message;
    for (int i = 0; i < 128 && runner_.EmitPending() != PlanningRunner::EmitResult::kDone; i++) {
        while (ring_.TryPop(out)) {}
    }
    EXPECT_FALSE(runner_.HasPending());
}

TEST_F(PlanningRunnerTest, ManualPipelineLimitsJointSpeed) {
    const ActionGroupSharedData group = MakeJointActionGroup();
    const FEEDBACK_ALL_JOINTS fb = ReadyFeedback();

    // 手动管线：L 域巡航速度 × 序列总长 = 物理速度，必须 ≤ 0.3 rad/s
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.PlanActionGroup(group, fb, PlanningRunner::GroupSource::kManual))
        << runner_.last_result().message;
    double manual_max = 0.0;
    {
        const action::PathActionSequence& seq = runner_.last_group().arms[kRightArmSoc];
        ASSERT_FALSE(seq.segments.empty());
        for (const auto& seg : seq.segments)
            manual_max = std::max(manual_max, seg.v_cruise * seq.total_length);
    }

    // 程序管线（独立 runner）：同一目标不受手动限速
    util::RingBufferIpcData<spline::PathSplineData, 64> prog_data{};
    util::RingBufferIpc<spline::PathSplineData, 64> prog_ring{&prog_data};
    PlanningRunner prog_runner;
    ASSERT_TRUE(prog_runner.Init(store_, &prog_ring));
    ASSERT_TRUE(prog_runner.PlanActionGroup(group, fb, PlanningRunner::GroupSource::kProgram))
        << prog_runner.last_result().message;
    double prog_max = 0.0;
    {
        const action::PathActionSequence& seq = prog_runner.last_group().arms[kRightArmSoc];
        ASSERT_FALSE(seq.segments.empty());
        for (const auto& seg : seq.segments)
            prog_max = std::max(prog_max, seg.v_cruise * seq.total_length);
    }

    EXPECT_LE(manual_max, 0.3 * (1.0 + 1e-6));
    EXPECT_GT(prog_max, manual_max * 2.0);  // 手动限速确实起了约束作用
}

TEST_F(PlanningRunnerTest, ChainedGroupStartsFromPreviousEnd) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    // 组1 全量发射且不弹出 → ② 非空，链记录 = 组1 终点
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()))
        << runner_.last_result().message;
    ASSERT_FALSE(ring_.IsEmpty());
    EXPECT_TRUE(runner_.IsChainValid());

    // 组2：目标再偏移 0.05→0.10；起点必须 = 组1 终点（而非反馈）
    ActionGroupSharedData g2 = MakeJointActionGroup();
    for (int j = 0; j < kArmJoints; ++j)
        g2.actions[kRightArmSoc].end_target.joints[j] = kReadyPose[j] + 0.10 * (j + 1);
    ASSERT_TRUE(runner_.PlanActionGroup(g2, ReadyFeedback()))
        << runner_.last_result().message;

    const action::PathActionSequence& seq = runner_.last_group().arms[kRightArmSoc];
    ASSERT_FALSE(seq.segments.empty());
    for (int j = 0; j < kArmJoints; ++j) {
        EXPECT_NEAR(seq.segments.front().start_target_data.joints[j],
                    kReadyPose[j] + 0.05 * (j + 1), 1e-9) << "joint " << j;
    }
    runner_.AbortPending();  // 清场，避免影响后续用例
}

TEST_F(PlanningRunnerTest, AbortPendingInvalidatesChain) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()));
    EXPECT_TRUE(runner_.IsChainValid());
    runner_.AbortPending();
    EXPECT_FALSE(runner_.IsChainValid());
}

TEST_F(PlanningRunnerTest, DeviationFromChainInsertsTransitionGroup) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    // 组1 全量发射后模拟执行完毕：排空 ②，链记录 = 组1 终点
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), ReadyFeedback()));
    spline::PathSplineData out{};
    while (ring_.TryPop(out)) {}

    // 反馈仍停了就绪姿态（≠ 链终点，偏离 > 1e-3 rad）→ 必须先补过渡组
    ActionGroupSharedData g2 = MakeJointActionGroup();
    for (int j = 0; j < kArmJoints; ++j)
        g2.actions[kRightArmSoc].end_target.joints[j] = kReadyPose[j] + 0.10 * (j + 1);
    ASSERT_TRUE(runner_.PlanActionGroup(g2, ReadyFeedback()))
        << runner_.last_result().message;

    // 全量排空发射（驱动 EmitPending 至 kDone）
    std::vector<spline::PathSplineData> emitted;
    for (int i = 0; i < 128 && runner_.EmitPending() != PlanningRunner::EmitResult::kDone; i++) {
        while (ring_.TryPop(out)) emitted.push_back(out);
    }
    while (ring_.TryPop(out)) emitted.push_back(out);
    EXPECT_FALSE(runner_.HasPending());

    // 主组段数（last_group 已是主组结果）
    const size_t main_count = runner_.last_group().arms[kRightArmSoc].segments.size();
    ASSERT_GE(main_count, 1u);
    // 过渡段存在：总发射段数 > 主组段数
    EXPECT_GT(emitted.size(), main_count);

    // 过渡段起点 = 反馈（就绪姿态）：用首段 joint_spline l=0 求值
    std::array<double, robot_config::kRobotMaxJoints> ones{};
    ones.fill(1.0);
    std::array<bool, robot_config::kRobotMaxJoints> all_true{};
    all_true.fill(true);
    spline::PathSpline first_path(&emitted.front());
    spline::JointArray q_start(ones, all_true);
    ASSERT_TRUE(first_path.joint_spline().Eval(0.0, q_start));
    for (int j = 0; j < kArmJoints; ++j)
        EXPECT_NEAR(q_start[j], kReadyPose[j], 1e-6) << "transition start joint " << j;

    // 主组首段起点 = 组1 终点（链接经过渡段闭合）
    const action::PathActionSequence& seq = runner_.last_group().arms[kRightArmSoc];
    for (int j = 0; j < kArmJoints; ++j)
        EXPECT_NEAR(seq.segments.front().start_target_data.joints[j],
                    kReadyPose[j] + 0.05 * (j + 1), 1e-9) << "main start joint " << j;
}

}  // namespace

// func R1/R2 验收（2026-09-09 交付）：全零起点（合法 URDF 零位）的关节指令
// 不再触发起点 IK，规划成功；段链连续、端点精确。
TEST_F(PlanningRunnerTest, ZeroFeedbackJointActionSucceeds) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    FEEDBACK_ALL_JOINTS zero_fb{};
    ASSERT_TRUE(runner_.ProcessActionGroup(MakeJointActionGroup(), zero_fb))
        << runner_.last_result().message;

    std::array<double, robot_config::kRobotMaxJoints> ones_data{};
    std::array<bool, robot_config::kRobotMaxJoints> all_true{};
    ones_data.fill(1.0);
    all_true.fill(true);
    int segment_count = 0;
    std::array<double, robot_config::kRobotMaxJoints> prev_end{};
    spline::PathSplineData out{};
    while (ring_.TryPop(out)) {
        ++segment_count;
        EXPECT_GT(out.path_time, 0.0);
        spline::PathSpline path(&out);
        spline::JointArray q_start(ones_data, all_true);
        spline::JointArray q_end(ones_data, all_true);
        ASSERT_TRUE(path.joint_spline().Eval(0.0, q_start));
        ASSERT_TRUE(path.joint_spline().Eval(1.0, q_end));
        if (segment_count == 1) {
            for (int j = 0; j < kArmJoints; ++j)
                EXPECT_NEAR(q_start[j], 0.0, 1e-6) << "start joint " << j;
        } else {
            for (int j = 0; j < robot_config::kRobotMaxJoints; ++j)
                EXPECT_NEAR(q_start[j], prev_end[j], 1e-9);
        }
        for (int j = 0; j < robot_config::kRobotMaxJoints; ++j)
            prev_end[j] = q_end[j];
    }
    EXPECT_GE(segment_count, 1);
    for (int j = 0; j < kArmJoints; ++j)
        EXPECT_NEAR(prev_end[j], kReadyPose[j] + 0.05 * (j + 1), 1e-6);
}

// FR2 验收（2026-09-17）：BODY 透传 unit（conf 注册 joint_passthrough）关节动作
// 可规划——joint 动作不需要位姿（FK 校验按动作类型放宽）
TEST_F(PlanningRunnerTest, BodyPassthroughJointActionSucceeds) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    constexpr int kBodySoc = 13;  // HumanoidSocUnitType::ROBOT_BODY
    const std::vector<int> body =
        UnitLogicalJoints(robot_config::RobotSocUnitType::ROBOT_BODY);
    ASSERT_EQ(body.size(), 4);

    FEEDBACK_ALL_JOINTS fb{};
    for (int j = 0; j < 4; ++j) fb.joints[body[j]].position_feedback = 0.0833;

    ActionGroupSharedData group{};
    ActionData& a = group.actions[kBodySoc];
    a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(kBodySoc);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    for (int j = 0; j < 4; ++j) t.joints[j] = 0.1333;
    t.joint_number = 4;
    t.is_joints_valid = true;
    group.soc_enable[kBodySoc] = true;

    ASSERT_TRUE(runner_.ProcessActionGroup(group, fb)) << runner_.last_result().message;
    const action::PathActionSequence& seq = runner_.last_group().arms[kBodySoc];
    ASSERT_FALSE(seq.segments.empty());
    for (const auto& seg : seq.segments)
        EXPECT_EQ(seg.type, action::SegmentType::kJoint);
    EXPECT_DOUBLE_EQ(seq.segments.front().v_enter, 0.0);  // fine point
    EXPECT_DOUBLE_EQ(seq.segments.back().v_exit, 0.0);
    for (int j = 0; j < 4; ++j)
        EXPECT_NEAR(seq.segments.back().end_target_data.joints[j], 0.1333, 1e-9);
}

// 防误用：透传 unit 的 LINE 动作必须被拒绝（位姿刻意无效，不得静默使用）
TEST_F(PlanningRunnerTest, BodyLineActionRejected) {
    ASSERT_TRUE(runner_.Init(store_, &ring_));
    constexpr int kBodySoc = 13;
    const std::vector<int> body =
        UnitLogicalJoints(robot_config::RobotSocUnitType::ROBOT_BODY);
    ASSERT_EQ(body.size(), 4);
    FEEDBACK_ALL_JOINTS fb{};
    for (int j = 0; j < 4; ++j) fb.joints[body[j]].position_feedback = 0.0833;

    ActionGroupSharedData group{};
    ActionData& a = group.actions[kBodySoc];
    a.motion_type = ACTION_MOTION_TYPE::ACTION_LINE;
    a.unit_type = static_cast<SOC_UNIT_TYPE>(kBodySoc);
    a.valid_start_target = false;
    TargetData& t = a.end_target;
    t.unit_type = a.unit_type;
    t.pose.pos.x = 0.1;  // 任意位姿目标
    t.pose.orient.q0 = 1.0;
    t.is_position_valid = true;
    t.is_orientation_valid = true;
    t.is_joints_valid = false;
    group.soc_enable[kBodySoc] = true;

    EXPECT_FALSE(runner_.ProcessActionGroup(group, fb));
    spline::PathSplineData out{};
    EXPECT_FALSE(ring_.TryPop(out));  // 队列无残留
}
