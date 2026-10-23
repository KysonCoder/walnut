/**
 * @file multi_arm_synchronizer.hpp
 * @brief 多臂时间同步器
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 *
 * (c)2026 Balance Beam Tech Ltd. All rights reserved.
 */

#ifndef __MULTI_ARM_SYNCHRONIZER_HPP__
#define __MULTI_ARM_SYNCHRONIZER_HPP__

#include "trajectory_types.hpp"
#include <vector>

namespace action {

class MultiArmSynchronizer {
public:
   struct ArmTrajectory {
      int arm_id{0};
      TrajectoryData traj;
      double natural_time{0.0};
   };

   enum SyncMode {
      kNone, kStartEndSync, kWaypointSync, kFullTimeSync
   };

   MultiArmSynchronizer() = default;
   ~MultiArmSynchronizer() = default;

   double FindSlowestArmTime(const std::vector<ArmTrajectory>& arms);
   void StretchToMatchTime(ArmTrajectory& arm, double target_time, SyncMode mode);
   bool Synchronize(std::vector<ArmTrajectory>& arms, SyncMode mode);

private:
   void ResampleTrajectory(TrajectoryData& traj, double time_scale);
   bool ValidateStretch(const TrajectoryData& traj, double original_time, double target_time);
};

} // namespace action

#endif // __MULTI_ARM_SYNCHRONIZER_HPP__
