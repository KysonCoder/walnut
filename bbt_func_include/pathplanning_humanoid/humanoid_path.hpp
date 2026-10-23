// func/pathplanning_humanoid/include/humanoid_path.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_HPP_

#include <cstddef>
#include <vector>

#include "math/quaternion.hpp"   // math::Cartesian3, math::Quaternion

namespace pphumanoid {

// 弧长参数化路径容器（纯几何，无速度/时间）。
// 老库 Path 用 section-L∈[0,2]；本类直接按弧长 l∈[0,GetTotalLength()] 插值，
// 供全身 TOPP 的弧长归一化（s = l / L_total）使用。
class HumanoidPath {
 public:
  struct Point {
    double arc_m = 0.0;
    math::Cartesian3<double> position;
    math::Quaternion<double> orient;
  };

  // points 至少 2 个且 arc_m 严格递增；否则返回 false。
  bool Build(std::vector<Point> points);
  bool IsValid() const { return points_.size() >= 2; }

  double GetTotalLength() const { return total_length_m_; }
  size_t GetPointCount() const { return points_.size(); }
  const std::vector<Point>& GetPoints() const { return points_; }

  // 按弧长插值；越界钳到端点。位置线性插值，姿态 slerp。
  bool Interpolate(double arc_m, math::Cartesian3<double>& pos_out,
                   math::Quaternion<double>& orient_out) const;

 private:
  std::vector<Point> points_;
  double total_length_m_ = 0.0;

  // 返回满足 points_[i].arc_m <= arc_m < points_[i+1].arc_m 的 i（末段取 size-2）
  size_t FindSegment_(double arc_m) const;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_HPP_
