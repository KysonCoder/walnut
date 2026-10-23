#ifndef TIME_OPTIMAL_PROFILER_HPP_
#define TIME_OPTIMAL_PROFILER_HPP_

#include "trajectory_types.hpp"
#include "path.hpp"
#include "dynamic_planner.hpp"
#include "math/se3_twist_ops.hpp"
#include <vector>

namespace action {

class TimeOptimalProfiler {
public:
    struct Profile {
        std::vector<double> L;      // path parameter samples
        std::vector<double> v;      // path velocity dL/dt
        std::vector<double> t;      // cumulative time (s)
        std::vector<double> a;      // path acceleration d²L/dt²
        bool feasible{false};       // true if a valid profile exists
    };

    struct Config {
        int num_samples;       // path sampling resolution
        double delta_L;        // finite difference step for derivatives
        Config() : num_samples(500), delta_L(1e-4) {}
    };

    explicit TimeOptimalProfiler(const Config& config = Config());

    // Main entry: plan time-optimal speed profile for a path
    bool Plan(const action::Path& path,
              pathplanning::DynamicPlanner& dyn,
              Profile& out);

    // Convert Profile to TrajectoryPoint sequence using Path geometry
    static bool ExportTrajectoryPoints(const Path& path,
                                       const Profile& profile,
                                       std::vector<TrajectoryPoint>& output);

private:
    Config config_;

    // Extract q, dq/dL, d²q/dL² at path sample L
    bool ExtractPathDerivatives_(const Path& path, double L,
                                 se3_twist::Vec6& q_out,
                                 se3_twist::Vec6& dq_dL_out,
                                 se3_twist::Vec6& d2q_dL2_out);

    // Compute acceleration bounds [L_bound, U] at sample with speed v
    bool ComputeBounds_(pathplanning::DynamicPlanner& dyn,
                        const se3_twist::Vec6& q,
                        const se3_twist::Vec6& dq_dL,
                        const se3_twist::Vec6& d2q_dL2,
                        double v,
                        double& L_bound_out, double& U_out);

    // Forward integration: maximum acceleration
    void ForwardPass_(const std::vector<double>& U,
                      double delta_L,
                      std::vector<double>& v_fwd);

    // Backward integration: maximum deceleration
    void BackwardPass_(const std::vector<double>& L_bound,
                       double delta_L,
                       std::vector<double>& v_bwd);

    // Clamp profile to velocity limit curve
    bool ClampToVelocityLimit_(pathplanning::DynamicPlanner& dyn,
                               const std::vector<se3_twist::Vec6>& q_samples,
                               const std::vector<se3_twist::Vec6>& dq_dL_samples,
                               const std::vector<se3_twist::Vec6>& d2q_dL2_samples,
                               std::vector<double>& v_profile);
};

} // namespace action

#endif
