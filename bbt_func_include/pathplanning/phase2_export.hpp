#ifndef FUNC_PATHPLANNING_INCLUDE_PHASE2_EXPORT_HPP_
#define FUNC_PATHPLANNING_INCLUDE_PHASE2_EXPORT_HPP_

#include <string>

#include "path_action_types.hpp"

namespace pathplanning {

/// Export per-SOC trajectory data from a PathActionGroup to JSON.
///
/// The generated JSON follows the schema expected by
/// unittest/plot_phase2_trajectories.py:
///   { "socs": [ { "name", "time", "cartesian":{x,y,z},
///                 "speed", "torque", "joint_pos", "joint_vel", "joint_acc" }, ... ] }
bool ExportPhase2ToJson(const action::PathActionGroup& result,
                        const std::string& filepath);

}  // namespace pathplanning

#endif  // FUNC_PATHPLANNING_INCLUDE_PHASE2_EXPORT_HPP_
