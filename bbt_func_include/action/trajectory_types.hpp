/**
 * @file trajectory_types.hpp
 * @brief Core data types for trajectory planning
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 */

#ifndef __TRAJECTORY_TYPES_HPP__
#define __TRAJECTORY_TYPES_HPP__

#include "legacy_robot_types.hpp"
#include "pose.hpp"
#include "quaternion.hpp"

#include <string>
#include <vector>

namespace action {

/**
 * @brief 轨迹规划错误码
 *
 * 错误码按类别分段：
 * - 1000段：输入错误
 * - 2000段：几何错误
 * - 3000段：奇异点错误
 * - 4000段：速度规划错误
 * - 5000段：多臂同步错误
 * - 9999：内部错误
 */
enum class PlanError {
   kOk = 0,                        ///< 成功
   kInvalidInput = 1001,           ///< 输入参数错误
   kUnsupportedMotionType = 1004,  ///< 管线不支持的动作类型（circle/force/spline/search）
   kZeroLengthSegment = 1002,      ///< 零长度路径段
   kTargetOutOfReach = 1003,       ///< 目标点不可达
   kCornerTooSharp = 2001,         ///< 拐角过锐
   kCornerZoneTooLarge = 2002,     ///< 拐角区过大
   kIKFailure = 2003,              ///< 逆运动学求解失败
   kCornerBlendFailed = 2004,      ///< 拐角混合（CornerBlender）失败
   kPersistentSingularity = 3001,  ///< 持续奇异点
   kSingularitySwitchFail = 3002,  ///< 奇异点空间切换失败
   kSegmentTooShortForAcc = 4001,  ///< 路径段过短无法满足加速度约束
   kJointSpeedExceeded = 4002,     ///< 关节速度超限
   kMultiArmTimeMismatch = 5001,   ///< 多臂时间不匹配
   kSyncStretchInfeasible = 5002,  ///< 时间拉伸不可行
   kInternalError = 9999           ///< 内部错误
};

/**
 * @brief 规划结果（错误或警告）
 */
struct PlanResult {
   PlanError code{PlanError::kOk};     ///< 错误码
   std::string message;                ///< 描述信息
   int arm_id{-1};                     ///< 关联臂ID（-1表示全局）
   size_t point_index{0};              ///< 关联轨迹点索引
};

/**
 * @brief 单点轨迹数据
 */
struct TrajectoryPoint {
   double timestamp{0.0};              ///< 时间戳（秒）
   SOC_JOINTS joints{};                ///< 各关节角（按SOC分组），默认全零
   math::Pose<double> tcp_pose;        ///< TCP位姿
   double tcp_speed{0.0};              ///< TCP线速度
   double tcp_acc{0.0};                ///< TCP线加速度
   double joint_speed{0.0};            ///< 归一化关节速度
   double joint_norm_vel[7]{};         ///< 各关节归一化速度（0~1，相对于关节最大速度）
   bool is_cartesian_space{false};     ///< 当前使用笛卡尔/关节空间
   double singularity_index{0.0};      ///< 可操作度（雅可比行列式）
};

/**
 * @brief 完整轨迹数据
 *
 * @note point_count 与 points.size() 应保持同步。point_count 提供
 *       显式缓存，避免频繁调用 size() 的隐式转换开销。
 */
struct TrajectoryData {
   std::vector<TrajectoryPoint> points;   ///< 轨迹点序列
   double total_time{0.0};                ///< 总时间（秒）
   int point_count{0};                    ///< 点数量（应与 points.size() 同步）
};

} // namespace action

#endif // __TRAJECTORY_TYPES_HPP__
