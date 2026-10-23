#ifndef PATH_ACTION_TYPES_HPP
#define PATH_ACTION_TYPES_HPP

#include "action.hpp"
#include "trajectory_types.hpp"
#include "math/se3_twist_ops.hpp"
#include <array>
#include <vector>

namespace action {

// kJoint: 关节空间直线段（2026-08-31 joint 动作支持）。
// joint 段 length_m 存关节路径长度 [rad]（语义按 type 解释），
// 两端 fine point（v_enter=v_exit=0），不参与拐角混合。
enum class SegmentType { kStraight, kJoint };

/// 段端点关节数组宽度: 统一 SOC_MAX_JOINT_NUMBER(=7)。
/// 2026-08-26 由定宽 6(se3_twist::Vec6)拓宽: humanoid 手臂 num_joints=7,
/// 定宽 6 导致第 7 关节 dq_dL 恒 0、Step4/TOPP 漏管(task 侧反馈 #3)。
/// ABI 破坏, 下游需重编。
using SegmentJointArray = std::array<double, SOC_MAX_JOINT_NUMBER>;

/// Per-endpoint dynamics data inside a PathActionSegment (POD-friendly)
/// 所有关节数组仅用前 num_joints 项, 其余恒 0。
struct SegmentEndpointDynamics {
    SegmentJointArray q_joints{};        // 关节角 [rad]
    SegmentJointArray dq_dL{};           // action 级关节差分 Δq [rad]:
                                         // 该 action 起止关节差, 同一 action 的
                                         // 所有子段取同一值(见 README 字段语义节)
    SegmentJointArray d2q_dL2{};         // d²q/dL²(归一化 L 域)
    SegmentJointArray mass_diag{};       // 质量阵对角(仅 pathplanning 动力学填充, ppwodyn 恒 0)
    SegmentJointArray torque_gravity{};  // 重力矩(同上)
    SegmentJointArray torque_coriolis{}; // 科氏矩(同上)
};

struct PathActionSegment {
    const CAction* action{nullptr};
    double L_start{0.0};   // 段起点, 整条序列归一化路径参数域坐标, L∈[0,1]
    double L_end{0.0};     // 段终点, 同域
    double length_m{0.0};  // 段物理长度：kStraight 为笛卡尔弧长 [m]，
                           // kJoint 为关节路径长度 [rad]（语义按 type 解释）
    SegmentType type{SegmentType::kStraight};

    double v_max_cartesian{0.0};     // 段级笛卡尔速度上限 [m/s]
    double v_max_joint_rad_s{0.0};   // 段级关节速度上限 [rad/s]

    // === Endpoint target data (POD, shareable across processes) ===
    TargetData start_target_data{};
    TargetData end_target_data{};

    // === Endpoint dynamics ===
    SegmentEndpointDynamics start_dyn{};
    SegmentEndpointDynamics end_dyn{};

    // === 时间参数化结果(Step5 填充) ===
    // v_enter/v_exit/v_cruise: 全局归一化 L 域的 dL/dt [1/s], 不是物理 m/s。
    // 换算: 物理速度 = v × total_length; 段局部(l∈[0,1])斜率 = v / (L_end - L_start)。
    // (task 侧 2026-08-26 实测经三角剖面闭合验证; ppwodyn fallback 路径
    //  2026-08-26 起同样统一为该域。)
    double v_enter{0.0};   // 段起点 dL/dt
    double v_exit{0.0};    // 段终点 dL/dt
    double v_cruise{0.0};  // 段巡航 dL/dt
    double timestamp{0.0}; // 段起点时刻 [s]。段时长 = 下一段 timestamp − 本段;
                           // 末段时长 = total_time − 本段 timestamp
};

struct SegmentSpeedInfo {
    const CAction* action{nullptr};
    double L_start{0.0};
    double L_end{0.0};
    double length_m{0.0};
    SegmentType type{SegmentType::kStraight};
    double v_max{0.0};
};

class PathActionSequence {
public:
    std::vector<PathActionSegment> segments;
    double total_length{0.0};  // 各段长度之和：纯 line 序列为 [m]；混合序列为
                               // m+rad 混合单位，仅作归一化 L 域权重，
                               // 见 docs/joint_line_normalization.md §2.3
    double total_time{0.0};
    int arm_id{0};

    const PathActionSegment* AtTime(double t) const;
    const PathActionSegment* AtLength(double s) const;
};

class PathActionGroup {
public:
    PathActionSequence arms[MAX_NO_OF_SOC];

    void GetJointPos(double time_ms, JOINTS_GROUP* joints_group);
    bool Execute(int engine_index);
    double GetTotalTime() const;
};

} // namespace action

#endif
