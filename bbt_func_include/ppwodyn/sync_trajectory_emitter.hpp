#ifndef FUNC_PPWODYN_INCLUDE_SYNC_TRAJECTORY_EMITTER_HPP_
#define FUNC_PPWODYN_INCLUDE_SYNC_TRAJECTORY_EMITTER_HPP_

#include "kinematic_topp_profiler.hpp"
#include <vector>

namespace ppwodyn {

// 同步轨迹发射器：把 profile 重采样到目标时长与固定采样数。
// 拷贝自 action::SyncTrajectoryEmitter（仅类型换成 ppwodyn Profile）。
class SyncTrajectoryEmitter {
 public:
  struct Result {
    std::vector<double> path_params;     // L(t)
    std::vector<double> velocities;      // dL/dt(t)
    std::vector<double> accelerations;   // d²L/dt²(t)
    std::vector<double> timestamps;      // t
    bool ok{false};
  };

  bool Emit(const KinematicToppProfiler::Profile& profile,
            double target_duration_s,
            int num_samples,
            Result& out) const;
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_SYNC_TRAJECTORY_EMITTER_HPP_
