#ifndef SYNC_TRAJECTORY_EMITTER_HPP
#define SYNC_TRAJECTORY_EMITTER_HPP
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "time_optimal_profiler.hpp"
#include <vector>

namespace action {

class SyncTrajectoryEmitter {
 public:
  struct Result {
    std::vector<double> path_params;     // L(t)
    std::vector<double> velocities;      // dL/dt(t)
    std::vector<double> accelerations;   // d²L/dt²(t)
    std::vector<double> timestamps;      // t
    bool ok{false};
  };

  // Resample a profile to a target duration with a fixed number of output samples.
  bool Emit(const TimeOptimalProfiler::Profile& profile,
            double target_duration_s,
            int num_samples,
            Result& out) const;
};

}  // namespace action

#endif  // SYNC_TRAJECTORY_EMITTER_HPP
#endif  // ROBOT_TYPE == DUAL_ARM
