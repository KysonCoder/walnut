#ifndef FUNC_PATHPLANNING_INCLUDE_PATH_PLANNING_LIMITS_ADAPTER_HPP_
#define FUNC_PATHPLANNING_INCLUDE_PATH_PLANNING_LIMITS_ADAPTER_HPP_

#include "velocity_profiler.hpp"
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM
#include "dynamic_planner.hpp"
#include "dual_arm_dyn.hpp"
#endif
#include "dynamics_config_adapter.hpp"

namespace pathplanning {

class PathPlanningLimitsAdapter {
 public:
  PathPlanningLimitsAdapter();
  ~PathPlanningLimitsAdapter();

  bool Attach(const robot_config::DynamicsConfigAdapter& adapter);
  bool IsAttached() const { return attached_; }

  // Fill velocity / acceleration limits from confbase.
  // Only overwrites fields that are currently 0 or equal to the library
  // hard-coded defaults. Positive user-provided values are preserved.
  bool FillVelocityLimits(action::VelocityProfiler::VelocityLimits* out) const;

#if ROBOT_TYPE == DUAL_ARM
  // Fill 13-DOF torque limits from drivechain tables.
  // Only fills if *out is effectively zero (unconfigured). User values are preserved.
  // Index 0 (floating base) is set to 1e9 to mean "passive / not enforced".
  bool FillTorqueLimits13(
      interpolation::Vec13* out,
      const pathplanning::DynamicPlanner::Config& dyn_config) const;
#endif

 private:
  bool attached_ = false;
};

}  // namespace pathplanning

#endif  // FUNC_PATHPLANNING_INCLUDE_PATH_PLANNING_LIMITS_ADAPTER_HPP_
