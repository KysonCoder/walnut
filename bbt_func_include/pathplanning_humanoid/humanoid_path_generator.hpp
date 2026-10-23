// func/pathplanning_humanoid/include/humanoid_path_generator.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_GENERATOR_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_GENERATOR_HPP_

#include <deque>
#include <vector>

#include "humanoid_path.hpp"
#include "pphumanoid_types.hpp"
#include "target.hpp"          // CTarget / TargetData / SOC_UNIT_TYPE

namespace pphumanoid {

// 拐角混合路径生成器：waypoints →（≥3 点时 CornerBlender 预采样拐角）→
// 等弧长直线采样 → HumanoidPath。
// 几何原语：action::CornerBlender（唯一拐角几何）+ kinematic::PathGeometry。
class HumanoidPathGenerator {
 public:
  struct BlendParams {
    double corner_radius_m = 0.01;
    double sample_step_m = 0.001;
  };

  // waypoints: 至少 2 个 CTarget（pose 有效即可，joints 不需要）。
  // waypoint_arcs_out[i] = 第 i 个原始路点在输出路径上的弧长边界
  // （拐角点映射到按比例折算的弧长），供管线段映射使用。
  int Generate(const std::vector<CTarget>& waypoints,
               const BlendParams& params,
               HumanoidPath& out,
               std::vector<double>& waypoint_arcs_out);

 private:
  // CornerBlender 输出 TargetData 的 backing store（deque 地址稳定）。
  // 成员持有，避免每次 Generate 重分配。
  std::deque<TargetData> blend_storage_;

  // 对（可能已混合的）路点序列做等弧长直线采样并累积弧长。
  int SamplePolyline_(const std::vector<CTarget>& polyline,
                      double sample_step_m,
                      std::vector<HumanoidPath::Point>& points_out) const;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_HUMANOID_PATH_GENERATOR_HPP_
