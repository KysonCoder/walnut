/**
 * @file singularity_detector.hpp
 * @brief 奇异点检测器
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 */

#ifndef __SINGULARITY_DETECTOR_HPP__
#define __SINGULARITY_DETECTOR_HPP__

#include "trajectory_types.hpp"
#include <vector>
#include <functional>
#include <array>

namespace action {

class SingularityDetector {
public:
   enum SingularityType {
      kNone, kShoulder, kElbow, kWrist
   };

   struct SingularityRegion {
      size_t point_start{0};
      size_t point_end{0};
      SingularityType type{kNone};
      double min_manipulability{0.0};
   };

   enum AvoidStrategy {
      kSwitchToJointSpace, kSlowDown, kReplan
   };

   struct RobotGeometry {
      double upper_arm_length{0.30};
      double forearm_length{0.30};
      double shoulder_wrist_max{0.60};
      double epsilon{1e-4};
   };

   using Jacobian = std::array<std::array<double, 6>, 6>;
   using ManipulabilityFunc = std::function<double(const SOC_JOINTS&)>;

   SingularityDetector();
   ~SingularityDetector();

   void SetRobotGeometry(const RobotGeometry& geom);
   void SetManipulabilityFunc(ManipulabilityFunc func);
   SingularityType CheckGeometricSingularity(const SOC_JOINTS& joints);
   double CalcManipulability(const SOC_JOINTS& joints);
   void ScanTrajectory(const std::vector<TrajectoryPoint>& points,
                       double threshold,
                       std::vector<SingularityRegion>& regions);
   AvoidStrategy RecommendStrategy(const SingularityRegion& region);

private:
   RobotGeometry geom_;
   ManipulabilityFunc manip_func_;
   double DefaultManipulability(const SOC_JOINTS& joints);
};

} // namespace action

#endif // __SINGULARITY_DETECTOR_HPP__
