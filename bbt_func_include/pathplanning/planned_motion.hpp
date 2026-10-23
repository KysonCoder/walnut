/**
 * @file planned_motion.hpp
 * @brief 规划运动输出结构
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 */

#ifndef __PLANNED_MOTION_HPP__
#define __PLANNED_MOTION_HPP__

#include "trajectory_types.hpp"
#include "singularity_detector.hpp"
#include "multi_arm_synchronizer.hpp"

#include <vector>
#include <iostream>

namespace action {

struct PlannedMotion {
   std::vector<MultiArmSynchronizer::ArmTrajectory> arms;
   std::vector<SingularityDetector::SingularityRegion> regions;
   double total_time{0.0};
   double sample_period{0.004};
   std::vector<PlanResult> errors;
   std::vector<PlanResult> warnings;

   bool GetPointAtIndex(size_t idx, int arm_id, TrajectoryPoint& out) const;
   bool GetPointAt(double t, int arm_id, TrajectoryPoint& out) const;
   bool HasErrors() const { return !errors.empty(); }
   bool HasWarnings() const { return !warnings.empty(); }
   void DumpReport(std::ostream& os) const;
};

} // namespace action

#endif
