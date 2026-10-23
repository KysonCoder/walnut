#ifndef DUAL_ARM_SYNCHRONIZER_HPP_
#define DUAL_ARM_SYNCHRONIZER_HPP_

#include "trajectory_types.hpp"
#include "dual_arm_dyn.hpp"
#include "dynamic_planner.hpp"
#include <vector>

namespace action {

class DualArmSynchronizer {
public:
    struct Config {
        se3_twist::Vec13 tau_max;
        int max_iterations{10};
        double time_match_eps{0.01};       // seconds
        double alpha_match{0.95};          // alpha reduction for time-matching
        double alpha_resolve{0.90};        // alpha reduction for violation resolution
        double base_slowdown_factor{1.05}; // T_target multiplier for base overload
        int coupling_samples{500};
    };

    struct Result {
        bool feasible{false};
        double final_makespan{0.0};
        int iterations_used{0};
        TrajectoryData traj_left;
        TrajectoryData traj_right;
    };

    DualArmSynchronizer() = default;

    // Main entry: find minimum feasible synchronized makespan
    bool Synchronize(const TrajectoryData& traj_left_in,
                     const TrajectoryData& traj_right_in,
                     interpolation::DualArmDyn& dual_dyn,
                     const Config& config,
                     Result& out);

private:
    bool SampleState_(const TrajectoryData& traj, double t,
                      se3_twist::Vec6& q, se3_twist::Vec6& qd,
                      se3_twist::Vec6& qdd) const;

    bool CheckCoupling_(const TrajectoryData& traj_L,
                        const TrajectoryData& traj_R,
                        double T_target,
                        interpolation::DualArmDyn& dual_dyn,
                        const se3_twist::Vec13& tau_max,
                        int samples,
                        int& worst_joint,
                        double& max_violation_ratio) const;
};

} // namespace action

#endif // DUAL_ARM_SYNCHRONIZER_HPP_
