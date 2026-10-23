/**
 * @file path.hpp
 * @brief 独立路径类（从 CSection 解耦）
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 *
 * 基于 PathGenerator 预计算路径，提供与 CSection 兼容的 Interpolate(L) 接口。
 * 不再依赖 CSection 的内部实现。
 */

#ifndef __PATH_HPP__
#define __PATH_HPP__

#include "target.hpp"
#include "path_generator.hpp"
#include "trajectory_types.hpp"
#include "velocity_profiler.hpp"

#include <vector>

namespace action {

/**
 * @brief 独立路径类
 *
 * 封装两段直线路径之间的拐角混合。预计算完整轨迹，
 * 提供与 CSection 兼容的 section L 参数插值。
 *
 * Section L 约定（与 CSection 一致）：
 * - L = 0.0: segm_2 起点（路径起点）
 * - L = 1.0: 拐角点
 * - L = 2.0: segm_1 终点（路径终点）
 */
class Path {
public:
   /**
    * @brief 构造路径
    *
    * @param[in] start segm_2 起点（对应 section L=0）
    * @param[in] mid   拐角点（对应 section L=1）
    * @param[in] end   segm_1 终点（对应 section L=2）
    * @param[in] sample_step_m 采样步长（米）
    */
   Path(const CTarget& start, const CTarget& mid, const CTarget& end,
        double sample_step_m = 0.01);

   /**
    * @brief 构造多段路径
    *
    * @param[in] waypoints 目标点序列（至少 2 个）
    * @param[in] sample_step_m 采样步长（米）
    */
   Path(const std::vector<CTarget>& waypoints, double sample_step_m = 0.01);

   ~Path() = default;

   /**
    * @brief 按 section L 参数插值一个点
    *
    * @param[in] section_L section 参数 [0.0, 2.0]
    * @param[out] out 输出目标点
    * @param[out] out_vel 可选：输出路径速度 (m/s)
    * @param[out] out_time 可选：输出累积时间 (s)
    * @param[out] out_joints 可选：输出关节角插值结果
    * @return true 表示成功
    */
   bool Interpolate(double section_L, CTarget& out,
                     double* out_vel = nullptr,
                     double* out_time = nullptr,
                     SOC_JOINTS* out_joints = nullptr) const;

   /**
    * @brief 计算速度剖面（需先完成路径构建）
    *
    * 使用 VelocityProfiler 对预计算路径进行 S-curve 速度规划。
    * 计算后每个 PathPoint 会填充 velocity 和 timestamp。
    *
    * @param[in] limits 速度/加速度/加加速度约束
    * @return true 表示成功
    */
   bool PlanSpeed(const VelocityProfiler::VelocityLimits& limits);

   /**
    * @brief 获取路径总点数
    */
   size_t GetPointCount() const { return points_.size(); }

   /**
    * @brief 获取拐角点索引
    */
   size_t GetCornerIndex() const { return corner_idx_; }

   /**
    * @brief 获取路径总长度（米）
    */
   double GetTotalLength() const { return total_length_; }

   /**
    * @brief 获取给定 L 处的局部距离缩放因子 d(弧长)/dL
    *
    * 利用 Path 内部点的 section_L 和 position 计算局部转换因子，
    * 替代全局平均值 total_length/2.0，提高两段长度差异大时的精度。
    *
    * @param[in] section_L section 参数 [0.0, 2.0]
    * @return 米每单位 L（始终为正）
    */
   double GetLocalLengthScale(double section_L) const;

   /**
    * @brief 获取路径总时间（秒，需先调用 PlanSpeed）
    */
   double GetTotalTime() const;

   /**
    * @brief 导出为 TrajectoryPoint 序列
    *
    * 将预计算路径点转换为 TrajectoryPoint，包含位置、姿态、速度、时间戳。
    * 需先调用 PlanSpeed 才能获取有效的时间/速度数据。
    *
    * @param[out] output 输出的 TrajectoryPoint 序列
    */
   void ExportTrajectoryPoints(std::vector<TrajectoryPoint>& output) const;

private:
   /**
    * @brief 带 section L 参数的轨迹点
    */
   struct PathPoint {
      double section_L{0.0};              ///< section 参数 [0, 2]
      math::Cartesian3<double> position;  ///< 位置
      math::Quaternion<double> orient;    ///< 姿态
      SOC_JOINTS joints{};                ///< 关节角（可选，由外部设置）
      bool has_joints{false};             ///< 关节角是否有效
      double velocity{0.0};               ///< 路径速度 (m/s)
      double timestamp{0.0};              ///< 累积时间 (s)
   };

   std::vector<PathPoint> points_;       ///< 预计算路径点
   size_t corner_idx_{0};                ///< 拐角点索引
   double total_length_{0.0};            ///< 路径总长度（米）

   SOC_UNIT_TYPE unit_type_{NO_UNIT};    ///< 路径点对应的单位类型

   /**
    * @brief 存储构造路径时输入 waypoints 的关节角
    *
    * 用于 Interpolate 时插值关节角。如果 waypoints 提供了有效关节角，
    * 则按 section_L 线性插值；否则 has_waypoint_joints_ = false。
    */
   std::vector<SOC_JOINTS> waypoint_joints_;
   std::vector<bool> waypoint_has_joints_;

   /**
    * @brief 根据 section_L 从 waypoint joints 插值关节角
    */
   bool InterpolateJoints_(double section_L, SOC_JOINTS& out) const;

   /**
    * @brief 构建单拐角路径（3 点）
    */
   void BuildPath(const CTarget& start, const CTarget& mid, const CTarget& end,
                  double sample_step_m);

   /**
    * @brief 构建多段路径（任意点数）
    */
   void BuildPath(const std::vector<CTarget>& waypoints, double sample_step_m);

   /**
    * @brief 查找包含 section_L 的点索引
    */
   size_t FindIndex(double section_L) const;
};

} // namespace action

#endif // __PATH_HPP__
