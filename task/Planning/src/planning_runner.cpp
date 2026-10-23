/**
 * @file planning_runner.cpp
 * @brief PlanningRunner 实现。
 */

#include "planning_runner.hpp"
#include "joint_pos.hpp"
#include "logger.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

PlanningRunner::PlanningRunner() : manual_pipeline_(MakeManualConfig()) {}

ppwodyn::PpwodynConfig PlanningRunner::MakeManualConfig() {
    // 手动管线限速（2026-09-08 手动模式设计 §3.1），其余字段保持默认。
    ppwodyn::PpwodynConfig cfg;
    cfg.vel_limits.v_max = kManualTcpSpeedLimitMps;
    cfg.vel_limits.v_joint_max = kManualJointSpeedLimitRadS;
    return cfg;
}

double PlanningRunner::BufferedTimeS() {
    if (traj_list_ == nullptr) return std::numeric_limits<double>::infinity();
    const size_t ring_size = traj_list_->Size();
    if (ring_size > emitted_time_fifo_.size()) {
        // 失账（Planning 重启后环内残留段）：按水位已满处理；
        // Servo 消费到 FIFO 可解释的区间后自动恢复。
        return std::numeric_limits<double>::infinity();
    }
    // 对账：Servo 每 TryPop 一段（含 Reset 排空、无效段丢弃）Size 即减一。
    while (emitted_time_fifo_.size() > ring_size) emitted_time_fifo_.pop_front();
    double total = 0.0;
    for (double t : emitted_time_fifo_) total += t;
    return total;
}

bool PlanningRunner::Init(robot_config::RobotConfigStore& store,
                          util::RingBufferIpc<spline::PathSplineData, 64>* traj_list) {
    if (!store.IsAttached() || traj_list == nullptr) {
        Log_E("PlanningRunner::Init: store not attached or null traj_list\n");
        return false;
    }
    if (!adapter_.Attach(store)) {
        Log_E("PlanningRunner::Init: DynamicsConfigAdapter attach failed\n");
        return false;
    }
    // base_link/body_link 为 ppwodyn 保留参数位，本库不使用。
    if (!pipeline_.Init(adapter_, "", "")) {
        Log_E("PlanningRunner::Init: PpwodynPipeline init failed\n");
        return false;
    }
    // 手动管线：同一适配器，仅限速不同（构造已注入手动配置）。
    if (!manual_pipeline_.Init(adapter_, "", "")) {
        Log_E("PlanningRunner::Init: manual pipeline init failed\n");
        return false;
    }
    // 与 CTaskFrame::MakeJointPos 相同的 typed-store → 全局布局 reinterpret。
    joint_map_ = reinterpret_cast<const robot_config::GlobalJointMap*>(
        store.GetGlobalJointMap());
    joint_params_ = reinterpret_cast<const robot_config::GlobalJointParams*>(
        store.GetGlobalJointParams());
    store_ = &store;
    traj_list_ = traj_list;
    start_group_.Attach(&start_data_);
    end_group_.Attach(&end_data_);
    initialized_ = true;
    return true;
}

bool PlanningRunner::BuildWaypointGroups(const ActionGroupSharedData& group,
                                         const FEEDBACK_ALL_JOINTS& feedback,
                                         bool force_chain_start) {
    bool any_enabled = false;
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc)
        if (group.soc_enable[soc]) any_enabled = true;
    if (!any_enabled) {
        Log_E("PlanningRunner: action group has no enabled SOC\n");
        return false;
    }

    // 先从 confbase 填充 TargetData（kinematic_model_name/kinematic_params 等
    // 为 CompositeRobotModel 必填）。注意：CTargetGroup::Detach 会清空共享数据，
    // 不要 Detach/re-Attach——视图在空数据上 Attach 后仍随数据填充正常工作
    // （2026-08-26 探针验证）。
    start_group_.InitializeFromRobot();
    end_group_.InitializeFromRobot();

    // FR2 透传 unit 激活（2026-09-17 探针实证）：InitializeFromRobot 只填拓扑链
    // unit（双臂）的槽位；conf 注册 joint_passthrough 的 unit 槽位由这里手工填充
    // 并按模型名路由 Attach（CTarget::Attach(unit_type, data) 按模型名优先路由）。
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc]) continue;
        const robot_config::UnitConfig* unit = nullptr;
        if (!adapter_.GetUnitConfig(soc, &unit) || unit == nullptr) {
            Log_E("PlanningRunner: no UnitConfig for soc %d\n", soc);
            return false;
        }
        if (strncmp(unit->kinematic_model_name, "joint_passthrough", 63) != 0) continue;
        for (CTargetGroup* grp : {&start_group_, &end_group_}) {
            TargetData& td = grp->GetSharedData()->targets[soc];
            td.unit_type = static_cast<SOC_UNIT_TYPE>(soc);
            strncpy(td.kinematic_model_name, unit->kinematic_model_name, 63);
            td.kinematic_model_name[63] = '\0';
            td.joint_number = unit->num_joints;
            CTarget& tgt = grp->GetTarget(soc);
            tgt.Detach();
            tgt.Attach(static_cast<SOC_UNIT_TYPE>(soc), &td);
        }
    }

    // 起点选择（2026-09-08 手动模式设计 §3.3）：② 有在途段（或过渡→主组原地
    // 续规划）时从链终点继续，保证段间位置连续；否则用反馈（原路径）。
    const bool use_chain_start = force_chain_start ||
        (chain_valid_ && traj_list_ != nullptr && traj_list_->Size() > 0);
    if (use_chain_start) {
        start_data_ = chain_end_data_;  // 含运动学字段与位姿缓存（TargetData POD 拷贝）
    } else {
    // 起点：反馈关节（全局数组）经 CJointPos 按单元映射，README 8.3 的
    // 标准流程是 SetSocJoints 写视图。
    double fb[robot_config::kRobotMaxJoints];
    for (int i = 0; i < robot_config::kRobotMaxJoints; ++i)
        fb[i] = feedback.joints[i].position_feedback;
    CJointPos joint_pos(fb, joint_map_, joint_params_);
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc]) continue;
        const robot_config::UnitConfig* unit = nullptr;
        if (!adapter_.GetUnitConfig(soc, &unit) || unit == nullptr) {
            Log_E("PlanningRunner: no UnitConfig for soc %d\n", soc);
            return false;
        }
        SOC_JOINTS sj{};
        for (int j = 0; j < unit->num_joints && j < SOC_MAX_JOINT_NUMBER; ++j)
            sj[j] = joint_pos.GetUnitJoint(j, unit);
        start_group_.GetTarget(soc).SetSocJoints(sj);
    }
    }

    // 终点：字段级叠加 end_target（保留 InitializeFromRobot 填的运动学字段）。
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc]) continue;
        // 2026-09-02 func R1：motion_type 权威字段在 ActionData，转写到终点
        // TargetData（RunAll 按终点类型分派）。task 侧提前校验以给出带 soc 的
        // 定位信息——NULL 到 func 层语义是"该 SOC 不动"，放行即静默丢失动作。
        const ACTION_MOTION_TYPE mt = group.actions[soc].motion_type;
        if (mt != ACTION_MOTION_TYPE::ACTION_LINE &&
            mt != ACTION_MOTION_TYPE::ACTION_JOINT) {
            Log_E("PlanningRunner: soc %d motion_type %d not plannable\n",
                  soc, static_cast<int>(mt));
            return false;
        }
        const TargetData& src = group.actions[soc].end_target;
        TargetData& dst = end_data_.targets[soc];
        dst.motion_type = mt;
        std::copy(src.joints, src.joints + SOC_MAX_JOINT_NUMBER, dst.joints);
        dst.joint_number = src.joint_number;
        dst.pose = src.pose;
        dst.config = src.config;
        dst.is_joints_valid = src.is_joints_valid;
        dst.is_position_valid = src.is_position_valid;
        dst.is_orientation_valid = src.is_orientation_valid;
    }

    // 最后设置使能/有效标志（前面的调用可能重置它们）。
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        start_data_.soc_enable[soc] = group.soc_enable[soc];
        end_data_.soc_enable[soc] = group.soc_enable[soc];
        if (group.soc_enable[soc]) {
            start_data_.targets[soc].is_joints_valid = true;
            // func CompactNullWaypoints 恒保留 waypoint[0] 并忽略其类型；
            // 显式置 ACTION_NULL 文档化"起点无插补类型"。
            start_data_.targets[soc].motion_type = ACTION_MOTION_TYPE::ACTION_NULL;
        }
    }

    // ppwodyn Step0 需要路径点位姿（IK 恢复/段几何）：起点由反馈关节 FK 得到，
    // 关节目标的终点同样补 FK。零工具系（Pose 默认构造即单位位姿）。
    // 2026-09-17 方案 A（FR2 透传适配）：全关节组不强制位姿有效——joint 动作的
    // 位置连续性由关节空间插值保证；透传 unit（无运动学模型）位姿刻意无效。
    // 含 LINE 的组维持位姿硬校验（line 段几何依赖位姿）。
    bool group_all_joint = true;
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (group.soc_enable[soc] &&
            group.actions[soc].motion_type != ACTION_MOTION_TYPE::ACTION_JOINT)
            group_all_joint = false;
    }
    FRAME tool_frame{};
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc]) continue;
        // CalcPose 返回值约定不明（实测成功=1）——以结果标志为准。
        start_group_.GetTarget(soc).CalcPose(&tool_frame, false);
        if (!start_data_.targets[soc].is_position_valid) {
            if (!group_all_joint) {
                Log_E("PlanningRunner: FK failed for start target (soc %d)\n", soc);
                return false;
            }
            Log_W("PlanningRunner: start pose unavailable (soc %d, joint group)\n", soc);
        }
        if (end_data_.targets[soc].is_joints_valid) {
            end_group_.GetTarget(soc).CalcPose(&tool_frame, false);
            if (!end_data_.targets[soc].is_position_valid) {
                if (!group_all_joint) {
                    Log_E("PlanningRunner: FK failed for end target (soc %d)\n", soc);
                    return false;
                }
                Log_W("PlanningRunner: end pose unavailable (soc %d, joint group)\n", soc);
            }
        }
    }
    return true;
}

bool PlanningRunner::ProcessActionGroup(const ActionGroupSharedData& group,
                                        const FEEDBACK_ALL_JOINTS& feedback) {
    // 兼容包装（无门控全量发射，程序管线）：保持旧语义，供既有单测。
    if (!PlanActionGroup(group, feedback, GroupSource::kProgram)) return false;
    for (int guard = 0; guard < 2; ++guard) {
        if (EmitImpl(false) == EmitResult::kError) return false;
        if (pending_phase_ != PendingPhase::kTransition) return true;
        pending_phase_ = PendingPhase::kMain;
        if (!RunPipelineForGroup(pending_group_, pending_feedback_, true)) return false;
    }
    return false;
}

bool PlanningRunner::PlanActionGroup(const ActionGroupSharedData& group,
                                     const FEEDBACK_ALL_JOINTS& feedback,
                                     GroupSource source) {
    if (!initialized_) {
        Log_E("PlanningRunner::PlanActionGroup: not initialized\n");
        return false;
    }
    if (pending_valid_) {
        Log_E("PlanningRunner::PlanActionGroup: pending emission not finished\n");
        return false;
    }
    pending_pipeline_ = (source == GroupSource::kManual) ? &manual_pipeline_ : &pipeline_;
    pending_group_ = group;
    pending_feedback_ = feedback;
    pending_phase_ = PendingPhase::kMain;
    if (NeedsTransition(group, feedback)) {
        // 反馈偏离链终点且 ② 已空：先补"反馈→链终点"过渡组（当前模式管线，
        // 手动模式同样受限速约束）
        pending_phase_ = PendingPhase::kTransition;
        return RunPipelineForGroup(SynthesizeTransitionGroup(group), feedback, false);
    }
    return RunPipelineForGroup(group, feedback, false);
}

bool PlanningRunner::RunPipelineForGroup(const ActionGroupSharedData& group,
                                         const FEEDBACK_ALL_JOINTS& feedback,
                                         bool force_chain_start) {
    if (!BuildWaypointGroups(group, feedback, force_chain_start)) return false;
    std::vector<const CTargetGroup*> waypoints{&start_group_, &end_group_};
    last_result_ = pending_pipeline_->RunAll(waypoints);
    if (!last_result_.ok) {
        Log_E("PlanningRunner: RunAll failed: %s (soc %d)\n",
              last_result_.message, last_result_.soc_index);
        return false;
    }
    pending_valid_ = true;
    pending_soc_ = 0;
    pending_seg_ = 0;
    return true;
}

bool PlanningRunner::NeedsTransition(const ActionGroupSharedData& group,
                                     const FEEDBACK_ALL_JOINTS& feedback) {
    if (!chain_valid_ || traj_list_ == nullptr || traj_list_->Size() > 0) return false;
    return !ChainMatchesFeedback(group, feedback);
}

bool PlanningRunner::ChainMatchesFeedback(const ActionGroupSharedData& group,
                                          const FEEDBACK_ALL_JOINTS& feedback) {
    double fb[robot_config::kRobotMaxJoints];
    for (int i = 0; i < robot_config::kRobotMaxJoints; ++i)
        fb[i] = feedback.joints[i].position_feedback;
    CJointPos joint_pos(fb, joint_map_, joint_params_);
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc] || !chain_end_data_.soc_enable[soc]) continue;
        const robot_config::UnitConfig* unit = nullptr;
        if (!adapter_.GetUnitConfig(soc, &unit) || unit == nullptr) return false;
        const TargetData& chain_end = chain_end_data_.targets[soc];
        if (!chain_end.is_joints_valid) return false;  // 无关节链终点 → 保守判偏离
        for (int j = 0; j < unit->num_joints && j < SOC_MAX_JOINT_NUMBER; ++j) {
            if (std::fabs(joint_pos.GetUnitJoint(j, unit) - chain_end.joints[j]) >=
                kChainMatchEpsRad) return false;
        }
    }
    return true;
}

ActionGroupSharedData PlanningRunner::SynthesizeTransitionGroup(
        const ActionGroupSharedData& group) const {
    ActionGroupSharedData transit{};  // 清零：速度等字段默认（管线内部限值生效）
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!group.soc_enable[soc] || !chain_end_data_.soc_enable[soc]) continue;
        transit.soc_enable[soc] = true;
        ActionData& a = transit.actions[soc];
        a.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
        a.unit_type = static_cast<SOC_UNIT_TYPE>(soc);
        a.valid_start_target = false;
        a.end_target = chain_end_data_.targets[soc];  // POD 拷贝（规划后关节+位姿）
        a.end_target.motion_type = ACTION_MOTION_TYPE::ACTION_JOINT;
        a.end_target.is_joints_valid = true;
    }
    return transit;
}

PlanningRunner::EmitResult PlanningRunner::EmitPending() {
    for (int guard = 0; guard < 2; ++guard) {  // 过渡→主组最多推进一相
        EmitResult r = EmitImpl(true);
        if (r != EmitResult::kDone) return r;
        if (pending_phase_ != PendingPhase::kTransition) return r;
        // 过渡组发完：原地规划主组（强制链起点=过渡组终点），随后继续按门控发射
        pending_phase_ = PendingPhase::kMain;
        if (!RunPipelineForGroup(pending_group_, pending_feedback_, true)) {
            AbortPending();
            return EmitResult::kError;
        }
    }
    Log_E("PlanningRunner::EmitPending: phase guard exceeded\n");
    return EmitResult::kError;
}

void PlanningRunner::ClearPendingState() {
    pending_valid_ = false;
    pending_soc_ = 0;
    pending_seg_ = 0;
}

void PlanningRunner::AbortPending() {
    ClearPendingState();
    emitted_time_fifo_.clear();
    chain_valid_ = false;
    pending_phase_ = PendingPhase::kMain;
    pending_pipeline_ = nullptr;
}

void PlanningRunner::UpdateChainEnd(const action::PathActionGroup& result) {
    for (int soc = 0; soc < MAX_NO_OF_SOC; ++soc) {
        if (!end_data_.soc_enable[soc]) continue;
        const action::PathActionSequence& seq = result.arms[soc];
        if (!seq.segments.empty())
            chain_end_data_.targets[soc] = seq.segments.back().end_target_data;
    }
    std::copy(end_data_.soc_enable, end_data_.soc_enable + MAX_NO_OF_SOC,
              chain_end_data_.soc_enable);
    chain_valid_ = true;
}

PlanningRunner::EmitResult PlanningRunner::EmitImpl(bool gated) {
    if (!pending_valid_) return EmitResult::kDone;
    // 段数据直接取 pending 管线结果：pending 期间不会再 RunAll（任务层门控 +
    // PlanActionGroup 防御），引用保持有效；PathActionSegment 含 CAction* 裸指针，
    // 不做整体拷贝。
    const action::PathActionGroup& result = pending_pipeline_->GetResult();
    for (int soc = pending_soc_; soc < MAX_NO_OF_SOC; ++soc) {
        if (!end_data_.soc_enable[soc]) { pending_seg_ = 0; continue; }
        const action::PathActionSequence& seq = result.arms[soc];
        // timestamp 语义（2026-08-26 探针实测）：段起点时刻。段时长 =
        // 下一段起点 − 本段起点；末段 = total_time − 起点。
        for (size_t i = pending_seg_; i < seq.segments.size(); ++i) {
            if (gated && BufferedTimeS() >= kTrajPrefetchTimeS) {
                pending_soc_ = soc;
                pending_seg_ = i;
                return EmitResult::kPaused;
            }
            const action::PathActionSegment& seg = seq.segments[i];
            const double next_t = (i + 1 < seq.segments.size())
                                      ? seq.segments[i + 1].timestamp
                                      : seq.total_time;
            const double seg_time = next_t - seg.timestamp;
            if (seg_time <= 0.0 || seg.length_m <= 0.0) {
                Log_E("PlanningRunner: degenerate segment (t=%f, L=%f)\n",
                      seg_time, seg.length_m);
                AbortPending();
                return EmitResult::kError;
            }
            // 本地构建完成后值拷贝进 shm 环——shm 内绝不出现半成品。
            spline::PathSplineData data{};
            if (!ConvertSegment(soc, seg, seg_time, &data)) {
                AbortPending();
                return EmitResult::kError;
            }
            if (!traj_list_->TryPush(data)) {
                Log_E("PlanningRunner: trajectory list full\n");
                AbortPending();
                return EmitResult::kError;
            }
            // 记账用拉伸后的实际段时长（ConvertSegment 峰值校验可能拉长 duration）。
            emitted_time_fifo_.push_back(data.path_time);
        }
        pending_seg_ = 0;
    }
    UpdateChainEnd(result);  // 发射完成：链终点 = 本组末段规划后终点
    ClearPendingState();
    return EmitResult::kDone;
}

bool PlanningRunner::ConvertSegment(int soc_index,
                                    const action::PathActionSegment& seg,
                                    double seg_time,
                                    spline::PathSplineData* out) {
    // 段类型分派（2026-09-01 kinplanning joint 支持）：
    //   kStraight — 笛卡尔直线段，length_m 为弧长 [m]
    //   kJoint    — 关节空间直线段，length_m 为关节路径长度 [rad]；
    //               端点关节线性插值与下方实现数学一致（割线 Hermite，dy=Δq），
    //               fine point（v_enter=v_exit=0）由库侧 TOPP 保证
    if (seg.type != action::SegmentType::kStraight &&
        seg.type != action::SegmentType::kJoint) {
        Log_E("PlanningRunner: unsupported segment type %d (soc %d)\n",
              static_cast<int>(seg.type), soc_index);
        return false;
    }
    const robot_config::UnitConfig* unit = nullptr;
    if (!adapter_.GetUnitConfig(soc_index, &unit) || unit == nullptr) {
        Log_E("PlanningRunner: no UnitConfig for soc %d\n", soc_index);
        return false;
    }
    const int nj = unit->num_joints;

    // 关节样条 q(l)，l∈[0,1]。kStraight/kJoint 均为 l 域直线，
    // 割线 Hermite（dy=Δq）精确成立，不使用 dq_dL（单位歧义，见计划头部）。
    double q0_raw[robot_config::kRobotMaxJoints]{};
    double q1_raw[robot_config::kRobotMaxJoints]{};
    double dq_raw[robot_config::kRobotMaxJoints]{};
    CJointPos jq0(q0_raw, joint_map_, joint_params_);
    CJointPos jq1(q1_raw, joint_map_, joint_params_);
    CJointPos jdq(dq_raw, joint_map_, joint_params_);
    // 关节角度取 TargetData.joints（7 宽）；start_dyn.q_joints 是定宽 6 的
    // se3_twist::Vec6，7 关节手臂越界读取是 UB（2026-08-26 核实）。
    for (int j = 0; j < nj && j < SOC_MAX_JOINT_NUMBER; ++j) {
        const double q0 = seg.start_target_data.joints[j];
        const double q1 = seg.end_target_data.joints[j];
        jq0.SetUnitJoint(j, q0, unit);
        jq1.SetUnitJoint(j, q1, unit);
        jdq.SetUnitJoint(j, q1 - q0, unit);  // l∈[0,1]，dq/dl = Δq
    }
    std::array<double, robot_config::kRobotMaxJoints> q0_arr{}, q1_arr{}, dq_arr{};
    std::array<bool, robot_config::kRobotMaxJoints> all_true{};
    all_true.fill(true);
    std::copy(q0_raw, q0_raw + robot_config::kRobotMaxJoints, q0_arr.begin());
    std::copy(q1_raw, q1_raw + robot_config::kRobotMaxJoints, q1_arr.begin());
    std::copy(dq_raw, dq_raw + robot_config::kRobotMaxJoints, dq_arr.begin());

    spline::PathSpline path(out);
    path.clear();
    path.SetMainLength(seg.length_m);
    path.SetSocPathLength(soc_index, seg.length_m);
    if (!path.joint_spline().CreateComplete(
            1.0,
            spline::JointArray(q0_arr, all_true),
            spline::JointArray(q1_arr, all_true),
            spline::JointArray(dq_arr, all_true),
            spline::JointArray(dq_arr, all_true))) {
        Log_E("PlanningRunner: joint spline build failed (soc %d)\n", soc_index);
        return false;
    }

    // 主样条 l(t)：t∈[0,T] → l∈[0,1]（段局部）。
    // v_enter/v_exit/v_cruise 是全局归一化 L 域的 dL/dt（2026-08-26 探针实测：
    // 全程 L∈[0,1]，5 段对称梯形），段局部斜率 = v / ΔL_global。
    // 单三次无法精确表达 TOPP 剖面：数值扫描峰值 dl/dt，超限则拉长 T 重建。
    const double dL_global = seg.L_end - seg.L_start;
    if (dL_global <= 0.0) {
        Log_E("PlanningRunner: degenerate L range (soc %d)\n", soc_index);
        return false;
    }
    const double v0 = seg.v_enter / dL_global;
    const double v1 = seg.v_exit / dL_global;
    const double v_peak_limit =
        (seg.v_cruise > 0.0 ? seg.v_cruise : seg.v_max_cartesian) / dL_global;
    double duration = seg_time;
    for (int iter = 0; iter < 4; ++iter) {
        if (!path.main_spline().create_complete(duration, 0.0, 1.0, v0, v1)) {
            Log_E("PlanningRunner: main spline build failed (soc %d)\n", soc_index);
            return false;
        }
        // 先写 path_time 再扫描：GetDldt 以 path_time 做域检查（此前漏写导致
        // 扫描静默失效——所有采样被判 out of range 返回 false，peak 恒 0）。
        path.SetPathTime(duration);
        double peak = 0.0;
        for (int k = 0; k <= 64; ++k) {
            double v = 0.0;
            if (path.GetDldt(duration * k / 64.0, v))
                peak = std::max(peak, v);
        }
        if (peak <= v_peak_limit * 1.001) break;
        duration *= peak / v_peak_limit;  // 等比拉长，端点物理速度不变
    }
    return true;
}
