/**
 * @file trajectory_planner.hpp
 * @brief 离线轨迹规划器（顶层入口）
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 */

#ifndef __TRAJECTORY_PLANNER_HPP__
#define __TRAJECTORY_PLANNER_HPP__

#include "planned_motion.hpp"
#include "path_generator.hpp"
#include "singularity_detector.hpp"
#include "velocity_profiler.hpp"
#include "time_optimal_profiler.hpp"
#include "dynamic_planner.hpp"
#include "multi_arm_synchronizer.hpp"
#include "dual_arm_synchronizer.hpp"
#include "trajectory_types.hpp"
#include "GenrArmDyn.hpp"
#include "dual_arm_dyn.hpp"

#include <vector>

namespace action {

class TrajectoryPlanner {
public:
   struct ArmInput {
      int arm_id{0};
      std::vector<CTarget> waypoints;
      PathGenerator::BlendMode blend_mode{PathGenerator::kCartesian};
   };

   struct Config {
      PathGenerator::BlendParams path_params;
      VelocityProfiler::VelocityLimits vel_limits;
      MultiArmSynchronizer::SyncMode sync_mode{MultiArmSynchronizer::kFullTimeSync};
      double singularity_threshold{0.01};
      bool use_time_optimal_profiler{false};
      bool use_dual_arm_time_optimal{false};
      pathplanning::DynamicPlanner::Config dyn_config;
      double left_torque_scale{1.0};
      double right_torque_scale{1.0};
   };

   TrajectoryPlanner();
   ~TrajectoryPlanner();

   // Set dynamics model for time-optimal profiling. Must be initialized before Plan().
   void SetDynamics(interpolation::GenrArmDyn* dyn) { genr_arm_dyn_ = dyn; }

   // Set dual-arm coupled dynamics model for synchronized planning.
   void SetDualArmDynamics(interpolation::DualArmDyn* dyn) { dual_arm_dyn_ = dyn; }

   PlanError Plan(const std::vector<ArmInput>& arms,
                  const Config& config,
                  PlannedMotion& result);

private:
   SingularityDetector sing_det_;
   MultiArmSynchronizer sync_;
   interpolation::GenrArmDyn* genr_arm_dyn_{nullptr};
   interpolation::DualArmDyn* dual_arm_dyn_{nullptr};

   PlanError PlanSingleArmPath(const ArmInput& input,
                               const Config& config,
                               MultiArmSynchronizer::ArmTrajectory& out);
};

} // namespace action

#endif
