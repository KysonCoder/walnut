/**
 * @file velocity_profiler.hpp
 * @brief S曲线速度规划器
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 */

#ifndef __VELOCITY_PROFILER_HPP__
#define __VELOCITY_PROFILER_HPP__

#include <vector>

namespace action {

class VelocityProfiler {
public:
   struct VelocityLimits {
      double v_max{0.25};
      double a_max{1.0};
      double j_max{10.0};
      double v_joint_max{1.05};
      double a_joint_max{1.05};
   };

   struct SegmentVelocity {
      double v_enter{0.0};
      double v_exit{0.0};
      double v_cruise{0.0};
      double t_acc{0.0};
      double t_cruise{0.0};
      double t_dec{0.0};
      double length{0.0};
   };

   VelocityProfiler() = default;
   ~VelocityProfiler() = default;

   void ForwardPass(std::vector<SegmentVelocity>& segs, const VelocityLimits& lim);
   void BackwardPass(std::vector<SegmentVelocity>& segs, const VelocityLimits& lim);
   bool GenerateProfile(SegmentVelocity& seg, const VelocityLimits& lim);
   double PlanPathSpeed(std::vector<SegmentVelocity>& segs, const VelocityLimits& lim);
};

} // namespace action

#endif // __VELOCITY_PROFILER_HPP__
