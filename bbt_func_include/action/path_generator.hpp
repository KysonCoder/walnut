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
 * - 保留既有 calc_corner_L 的线性+正弦混合算法，仅重组调用顺序
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
    * @brief 混合模式
    */
   enum BlendMode {
      kCartesian,    ///< 笛卡尔空间混合
      kJoint,        ///< 关节空间混合
      kAuto          ///< 自动选择（基于奇异点检测结果）
   };

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
    * @brief 笛卡尔空间拐角混合
    *
    * 使用线性+正弦权重混合两段直线路径。
    * 公式来源：既有 calc_corner_L 的实现。
    *
    * @param[in] start 起始目标点
    * @param[in] mid   拐角目标点（两段交点）
    * @param[in] end   终止目标点
    * @param[in] params 混合参数
    * @param[out] blended_path 输出的混合路径点序列（仅位置+姿态，无IK）
    * @return PlanError 错误码
    */
   PlanError BlendCartesianSegments(
      const CTarget& start,
      const CTarget& mid,
      const CTarget& end,
      const BlendParams& params,
      std::vector<PathInterpolationPoint>& blended_path);

   /**
    * @brief 关节空间拐角混合（双权重版）
    *
    * 使用双独立权重的加法混合公式，数学等价于 section.cpp
    * 的 target_add_sub_tcprobot：
    *   tr = L1 * segm1 + (1-L2) * segm2 + (L2-L1) * corner
    *
    * 当 L1 = L2 = w 时退化为单权重线性插值 w*segm1 + (1-w)*segm2。
    *
    * @param[in] segm1_joints  第一段终点关节角
    * @param[in] segm2_joints  第二段起点关节角
    * @param[in] corner_joints 拐角点关节角
    * @param[in] L1            进入混合深度 [0,1]
    * @param[in] L2            离开混合深度 [0,1]
    * @param[out] out          混合后关节角
    */
   static void BlendJointSegments(
      const SOC_JOINTS& segm1_joints,
      const SOC_JOINTS& segm2_joints,
      const SOC_JOINTS& corner_joints,
      double L1, double L2,
      SOC_JOINTS& out);

   /**
    * @brief 关节空间拐角混合（采样版）
    *
    * 在拐角区内按 sample_step 采样，内部使用双权重公式
    * （L1 = 1-w, L2 = w, w = sin(pi*t/2)）。
    *
    * @param[in] segm1_joints 第一段终点关节角
    * @param[in] segm2_joints 第二段起点关节角
    * @param[in] corner_joints 拐角点关节角
    * @param[in] params 混合参数
    * @param[out] blended_joints 输出的混合关节序列
    * @return PlanError 错误码
    */
   PlanError BlendJointSegments(
      const SOC_JOINTS& segm1_joints,
      const SOC_JOINTS& segm2_joints,
      const SOC_JOINTS& corner_joints,
      const BlendParams& params,
      std::vector<SOC_JOINTS>& blended_joints);

   /**
    * @brief 单段直线插值（笛卡尔空间）
    *
    * 在两点之间生成等距采样点，仅做笛卡尔插值（不做IK）。
    * 这是 BlendCartesianSegments 的底层 building block。
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
    * @brief 主入口：根据模式生成完整混合路径
    *
    * 输入为 CTarget 序列（每点代表一段直线运动端点），
    * 输出为带时间戳的 TrajectoryPoint 序列。
    *
    * @param[in] actions 目标点序列（每段一个 CTarget）
    * @param[in] mode    混合模式
    * @param[in] params  混合参数
    * @param[out] output 输出轨迹点序列
    * @return PlanError 错误码
    */
   PlanError GenerateBlendedPath(
      const std::vector<CTarget>& actions,
      BlendMode mode,
      const BlendParams& params,
      std::vector<TrajectoryPoint>& output);

private:
   /**
    * @brief 计算拐角混合权重（线性+正弦）
    *
    * 来自既有 calc_corner_L 的公式：
    * - 线性：w = (L - Lx2) / (Lx1 - Lx2)
    * - 正弦：w = sin(pi * w_linear / 2)
    *
    * @param[in] L   当前路径参数
    * @param[in] Lx1 拐角区起始边界
    * @param[in] Lx2 拐角区终止边界
    * @param[out] w1 第一段权重
    * @param[out] w2 第二段权重
    */
   void CalcBlendWeights(
      double L, double Lx1, double Lx2,
      double& w1, double& w2) const;

   /**
    * @brief 使用正弦权重混合两个位置
    */
   math::Cartesian3<double> BlendPosition(
      const math::Cartesian3<double>& p1,
      const math::Cartesian3<double>& p2,
      double w1, double w2) const;

   /**
    * @brief 使用正弦权重混合两个姿态
    */
   math::Quaternion<double> BlendOrientation(
      const math::Quaternion<double>& q1,
      const math::Quaternion<double>& q2,
      double w1, double w2) const;

   std::unique_ptr<interpolation::CKinGeneRbt> kin_;     ///< 运动学求解器
};

} // namespace action

#endif // __PATH_GENERATOR_HPP__
