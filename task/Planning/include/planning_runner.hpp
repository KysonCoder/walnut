/**
 * @file planning_runner.hpp
 * @brief ppwodyn 离线轨迹规划执行器：ActionGroupSharedData → 逐段 PathSplineData。
 *
 * 非 shm 驻留的进程内对象。动作触发的非周期路径（Planning MainLoop 64ms），
 * 内部经 ppwodyn 使用 STL/堆分配——不在 1kHz 热路径上。
 */

#ifndef __PLANNING_RUNNER_HPP__
#define __PLANNING_RUNNER_HPP__

#include <deque>
#include <limits>

#include "action/actiondata.hpp"
#include "ppwodyn/ppwodyn_pipeline.hpp"
#include "spline/path_spline.hpp"
#include "spline/spline_data.hpp"
#include "utilbase/ring_buffer_ipc.hpp"
#include "confbase/dynamics_config_adapter.hpp"
#include "confbase/robot.hpp"
#include "targetgroup.hpp"
#include "tf_types.hpp"

class PlanningRunner {
public:
    PlanningRunner();

    // store 必须已 attach；traj_list 为 trajectory_list_ 环形队列（注入以便单测）。
    bool Init(robot_config::RobotConfigStore& store,
              util::RingBufferIpc<spline::PathSplineData, 64>* traj_list);
    bool IsInitialized() const { return initialized_; }

    // ② traj_list 水位门控阈值：1.5 × Planning 主周期 64ms
    // （参考 ref CInterpolatorEngine::IsInterpolate 的 PREFETCH_TIME 思想）。
    static constexpr double kTrajPrefetchTimeS = 0.096;

    // 手动限速（ref programengine.cpp 注释的 _jogging_spd={0.1,0.3,…}，
    // SPEED_DATA 字段顺序 {tcp_m_s, joint_rad_s}）
    static constexpr double kManualTcpSpeedLimitMps = 0.1;
    static constexpr double kManualJointSpeedLimitRadS = 0.3;

    enum class GroupSource { kProgram, kManual };

    static constexpr double kChainMatchEpsRad = 1e-3;  // 反馈≈链终点判据（每关节）

    // 环内缓冲运动时间（秒）= FIFO 记账之和，经 ring Size() 对账。
    // 失账（进程重启后环内残留段，Size > FIFO 长度）返回 +inf，按水位已满处理。
    double BufferedTimeS();

    // 规划一个动作组并逐段 push 轨迹。失败返回 false，last_result() 含诊断。
    bool ProcessActionGroup(const ActionGroupSharedData& group,
                            const FEEDBACK_ALL_JOINTS& feedback);

    enum class EmitResult { kDone, kPaused, kError };

    // 规划一个动作组进入待发射状态（BuildWaypointGroups + RunAll，不发射）。
    // 失败返回 false（last_result() 含诊断）；有待发射残余时防御性拒绝。
    bool PlanActionGroup(const ActionGroupSharedData& group,
                         const FEEDBACK_ALL_JOINTS& feedback,
                         GroupSource source = GroupSource::kProgram);

    // 按水位门控发射待发射段：BufferedTimeS() ≥ kTrajPrefetchTimeS 即暂停并
    // 保存游标（下周期续传）。kError = 段退化/环满（待发射状态已中止）。
    EmitResult EmitPending();

    bool HasPending() const { return pending_valid_; }

    // ERROR/MotorOff：丢弃待发射状态与记账 FIFO（环由 Servo Reset 排空，
    // 对账自愈；排空前 BufferedTimeS 按失账返回 +inf，发射保持暂停）。
    void AbortPending();

    // 链记录是否有效（上一组终点可作下一组起点）。AbortPending/Init 后为 false。
    bool IsChainValid() const { return chain_valid_; }

    const ppwodyn::PpwodynStepResult& last_result() const { return last_result_; }

    // 最近一次 RunAll 的规划结果（段类型/速度剖面），供测试断言段级语义。
    const action::PathActionGroup& last_group() const {
        return (pending_pipeline_ != nullptr ? *pending_pipeline_ : pipeline_).GetResult();
    }

    // public 以便单测直构段验证段转换语义；生产路径一律走 ProcessActionGroup。
    bool ConvertSegment(int soc_index,
                        const action::PathActionSegment& seg,
                        double seg_time,
                        spline::PathSplineData* out);

private:
    // 手动管线配置（构造注入；PpwodynPipeline 不可赋值——unique_ptr 成员）
    static ppwodyn::PpwodynConfig MakeManualConfig();

    enum class PendingPhase { kTransition, kMain };

    bool BuildWaypointGroups(const ActionGroupSharedData& group,
                             const FEEDBACK_ALL_JOINTS& feedback,
                             bool force_chain_start);
    // BuildWaypointGroups + RunAll + 置 pending 游标（统一入口）
    bool RunPipelineForGroup(const ActionGroupSharedData& group,
                             const FEEDBACK_ALL_JOINTS& feedback,
                             bool force_chain_start);
    // 过渡段判定：链有效 且 ② 空 且 反馈偏离链终点
    bool NeedsTransition(const ActionGroupSharedData& group,
                         const FEEDBACK_ALL_JOINTS& feedback);
    bool ChainMatchesFeedback(const ActionGroupSharedData& group,
                              const FEEDBACK_ALL_JOINTS& feedback);
    ActionGroupSharedData SynthesizeTransitionGroup(const ActionGroupSharedData& group) const;
    EmitResult EmitImpl(bool gated);   // gated=false：无门控全量发射（兼容语义）
    void ClearPendingState();          // 发完/中止共用：清游标；不动 FIFO
    // 更新链终点 = 刚发完组的末段规划后终点（EmitImpl kDone 调用）
    void UpdateChainEnd(const action::PathActionGroup& result);

    ppwodyn::PpwodynPipeline pipeline_;
    ppwodyn::PpwodynPipeline manual_pipeline_;      // 手动限速配置（Init 时构造注入）
    ppwodyn::PpwodynPipeline* pending_pipeline_ = nullptr;  // 本次 pending 使用的管线
    robot_config::DynamicsConfigAdapter adapter_;
    robot_config::RobotConfigStore* store_ = nullptr;
    util::RingBufferIpc<spline::PathSplineData, 64>* traj_list_ = nullptr;

    // CJointPos 全局映射所需（Init 时从 store 缓存）。
    const robot_config::GlobalJointMap* joint_map_ = nullptr;
    const robot_config::GlobalJointParams* joint_params_ = nullptr;

    // CTargetGroup 视图指向的持久 POD 存储。
    TargetGroupSharedData start_data_{};
    TargetGroupSharedData end_data_{};
    CTargetGroup start_group_;
    CTargetGroup end_group_;

    ppwodyn::PpwodynStepResult last_result_{};
    bool initialized_ = false;

    // 已推入 ② 且未被对账的段时长（秒）；长度 ≤ 环容量 64，与环内段一一对应。
    // 元素为 ConvertSegment 峰值校验拉伸后的实际 path_time。
    std::deque<double> emitted_time_fifo_;

    bool pending_valid_ = false;
    int pending_soc_ = 0;              // 续传游标：外层 soc
    size_t pending_seg_ = 0;           // 续传游标：组内段序号

    TargetGroupSharedData chain_end_data_{};  // 上一组各 enabled SOC 的规划后终点
    bool chain_valid_ = false;

    ActionGroupSharedData pending_group_{};     // 过渡相期间持有的用户组（POD 副本）
    FEEDBACK_ALL_JOINTS pending_feedback_{};    // 主组原地续规划用的反馈快照
    PendingPhase pending_phase_ = PendingPhase::kMain;
};

#endif // __PLANNING_RUNNER_HPP__
