/**
 * @file trajectory_planner.hpp
 * @brief 离线轨迹规划器（顶层入口）
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-05-11
 */

#ifndef __TRAJECTORY_PLANNER_HPP__
#define __TRAJECTORY_PLANNER_HPP__

#include "confbase/robot.hpp"
#include "planned_motion.hpp"
#include "path_generator.hpp"
#include "singularity_detector.hpp"
#include "velocity_profiler.hpp"
#include "time_optimal_profiler.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "dynamic_planner.hpp"
#endif
#include "multi_arm_synchronizer.hpp"
#include "dual_arm_synchronizer.hpp"
#include "trajectory_types.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "GenrArmDyn.hpp"
#include "dual_arm_dyn.hpp"
#endif
#include "dynamics_config_adapter.hpp"
#include "kinematics_resolver.hpp"
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "nie_group.hpp"
#endif
#include "path_planning_limits_adapter.hpp"


#include <vector>
#include <memory>
#include <cstring>

namespace action {

class TrajectoryPlanner {
public:
   struct ArmInput {
      int arm_id{0};
      std::vector<CTarget> waypoints;
   };

   struct Config {
      PathGenerator::BlendParams path_params;
      VelocityProfiler::VelocityLimits vel_limits;
      MultiArmSynchronizer::SyncMode sync_mode{MultiArmSynchronizer::kFullTimeSync};
      double singularity_threshold{0.01};
      bool use_time_optimal_profiler{false};
      bool use_dual_arm_time_optimal{false};
#if ROBOT_TYPE == DUAL_ARM
      pathplanning::DynamicPlanner::Config dyn_config;
#endif
      double left_torque_scale{1.0};
      double right_torque_scale{1.0};
   };

   TrajectoryPlanner();
   ~TrajectoryPlanner();

   // Set dynamics model for time-optimal profiling. Must be initialized before Plan().
   bool Init(const robot_config::DynamicsConfigAdapter& adapter,
             robot_config::RobotSocUnitType unit_type,
             const std::string& root_link,
             const std::string& tip_link);

#if ROBOT_TYPE == DUAL_ARM
   // Initialize dual-arm synchronizer for coupled 13-DOF dynamics checking.
   bool InitDualArm(const robot_config::DynamicsConfigAdapter& adapter,
                    robot_config::RobotSocUnitType left_unit,
                    robot_config::RobotSocUnitType right_unit,
                    const std::string& base_link,
                    const std::string& body_link);
#endif

   bool IsInitialized() const { return initialized_; }

#if ROBOT_TYPE == DUAL_ARM
   const interpolation::GenrArmDyn* GetDynamics() const {
      return genr_arm_dyn_.get();
   }
#endif

   PlanError Plan(const std::vector<ArmInput>& arms,
                  const Config& config,
                  PlannedMotion& result);

private:
   SingularityDetector sing_det_;
   MultiArmSynchronizer sync_;
#if ROBOT_TYPE == DUAL_ARM
   std::unique_ptr<interpolation::GenrArmDyn> genr_arm_dyn_;
   std::unique_ptr<DualArmSynchronizer> dual_arm_sync_;
#endif
   std::unique_ptr<KinematicsResolver> kinematics_resolver_;
#if ROBOT_TYPE == DUAL_ARM
   std::unique_ptr<CNieGroup> nie_group_;
   std::unique_ptr<TargetGroupSharedData> nie_data_;
#endif
   pathplanning::PathPlanningLimitsAdapter limits_adapter_;
   bool initialized_ = false;

   PlanError PlanSingleArmPath(const ArmInput& input,
                               const Config& config,
                               MultiArmSynchronizer::ArmTrajectory& out);
};

} // namespace action

#endif
