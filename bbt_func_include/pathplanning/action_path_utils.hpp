/**
 * @file action_path_utils.hpp
 * @brief Utility functions for path planning on CAction objects
 * @author Claude
 * @version 0.1
 * @date 2026-05-27
 *
 * These functions were extracted from CAction to eliminate the action -> pathplanning
 * dependency. They live in pathplanning and operate on CAction through its public interface.
 */

#ifndef __ACTION_PATH_UTILS_HPP__
#define __ACTION_PATH_UTILS_HPP__

#include "action.hpp"
#include "trajectory_types.hpp"
#include "path.hpp"

#include <vector>

namespace pathplanning {

/**
 * @brief 单点插值（替代 CAction::InterpolatePoint）
 *
 * 在 action 的起始和终止目标点之间按归一化参数 L 插值一个点。
 *
 * @param[in] action 输入动作
 * @param[in] L 归一化路径参数 [0.0, 1.0]
 * @param[out] out_target 输出目标点
 * @param[out] out_method 使用的插值方法
 * @return true 表示成功
 */
bool InterpolateActionPoint(
    CAction& action,
    double L,
    CTarget& out_target,
    interpolation::path_ipol_method& out_method);

/**
 * @brief 离线生成 action 全部采样点（替代 CAction::GenerateTrajectoryPoints）
 *
 * 生成从 start 到 end 的完整 TrajectoryPoint 序列。
 *
 * @param[in] action 输入动作
 * @param[in] sample_step_m 采样步长（米）
 * @param[out] out 输出轨迹点序列
 * @return true 表示成功
 */
bool GenerateActionTrajectoryPoints(
    CAction& action,
    double sample_step_m,
    std::vector<action::TrajectoryPoint>& out);

} // namespace pathplanning

#endif // __ACTION_PATH_UTILS_HPP__
