/**
 * @file path_generator.hpp
 * @brief 路径生成器 — 为轨迹规划生成混合路径
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 *
 * 关键设计约束：
 * - 拐角几何由 CornerBlender（parabolic+sine 预采样）统一生成
 * - 基于 func/kinematic (PathGeometry + CKinGeneRbt)，替换 interpolate_pos 内嵌逻辑
 * - 输出 TrajectoryPoint 序列供后续 VelocityProfiler / SingularityDetector 使用
 */

#ifndef __PATH_GENERATOR_HPP__
#define __PATH_GENERATOR_HPP__

#include "trajectory_types.hpp"
#include "actiondata.hpp"       // for PathInterpolationPoint
#include "target.hpp"
#include "path_geometry.hpp"
#include "quaternion.hpp"       // for math::Cartesian3, math::Quaternion

#include <vector>
#include <memory>

// Forward declaration
namespace interpolation {
class CKinGeneRbt;
}

namespace action {

/**
 * @brief 路径生成器
 *
 * 为直线段之间的拐角生成混合路径。支持笛卡尔空间、关节空间及自动模式。
 * 所有几何计算委托给 PathGeometry，所有运动学计算委托给 CKinGeneRbt。
 */
class PathGenerator {
public:
   /**
    * @brief 混合参数
    */
   struct BlendParams {
      double corner_radius_m{0.01};      ///< 拐角区半径（米）
      double sample_step_m{0.001};       ///< 采样步长（米）
   };

   PathGenerator();
   ~PathGenerator();

   /**
    * @brief 单段直线插值（笛卡尔空间）
    *
    * 在两点之间生成等距采样点，仅做笛卡尔插值（不做IK）。
    *
    * @param[in] start 起始目标点
    * @param[in] end   终止目标点
    * @param[in] sample_step_m 采样步长（米）
    * @param[out] points 输出插值点序列
    * @return PlanError 错误码
    */
   PlanError InterpolateLineSegment(
      const CTarget& start,
      const CTarget& end,
      double sample_step_m,
      std::vector<PathInterpolationPoint>& points);

   /**
    * @brief 主入口：生成完整混合路径
    *
    * 输入为 CTarget 序列（每点代表一段直线运动端点），
    * 输出为带时间戳的 TrajectoryPoint 序列。
    * 拐角由 CornerBlender（parabolic+sine 预采样）消解。
    *
    * @param[in] actions 目标点序列（每段一个 CTarget）
    * @param[in] params  混合参数
    * @param[out] output 输出轨迹点序列
    * @return PlanError 错误码
    */
   PlanError GenerateBlendedPath(
      const std::vector<CTarget>& actions,
      const BlendParams& params,
      std::vector<TrajectoryPoint>& output);

private:
   std::unique_ptr<interpolation::CKinGeneRbt> kin_;     ///< 运动学求解器
};

} // namespace action

#endif // __PATH_GENERATOR_HPP__
