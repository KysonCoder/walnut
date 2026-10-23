#ifndef TIME_OPTIMAL_PROFILER_HPP_
#define TIME_OPTIMAL_PROFILER_HPP_
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "trajectory_types.hpp"
#include "path.hpp"
#include "dynamic_planner.hpp"
#include "math/se3_twist_ops.hpp"
#include <vector>

// Forward declaration
namespace action {
class PathActionSequence;
}

namespace action {

class TimeOptimalProfiler {
public:
    struct Profile {
        std::vector<double> L;      // path parameter samples
        std::vector<double> v;      // path velocity dL/dt
        std::vector<double> t;      // cumulative time (s)
        std::vector<double> a;      // path acceleration d²L/dt²
        std::vector<se3_twist::Vec6> q;  // per-sample joint angles (after IK recovery, if any)
        bool feasible{false};       // true if a valid profile exists
        bool used_fallback{false};  // true if fallback trapezoidal profile was used
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

    // PathActionSequence overload
    bool Plan(const PathActionSequence& seq,
              pathplanning::DynamicPlanner& dyn,
              Profile& out);

    // Convert Profile to TrajectoryPoint sequence using Path geometry
    bool ExportTrajectoryPoints(const Path& path,
                                const Profile& profile,
                                std::vector<TrajectoryPoint>& output);

    void SetFallbackLimits(double v_max, double a_max) {
        if (v_max > 0.0 && a_max > 0.0) {
            fallback_v_max_ = v_max;
            fallback_a_max_ = a_max;
        }
    }

    // Set kinematic context for per-sample IK recovery via CTarget::CalcJoint.
    void SetKinematicContext(const MAN_ROBOT& robot_param, const FRAME& tool_frame) {
        robot_param_ = robot_param;
        tool_frame_ = tool_frame;
        has_kinematic_context_ = true;
    }
    void SetIkRecoveryEnabled(bool enabled) { ik_recovery_enabled_ = enabled; }

    // Seed the first IK recovery (e.g. from the path's start waypoint joints).
    // Without a seed the first solve uses the solver's default branch.
    // The seed is stored as an external seed and re-applied at the start of
    // every Plan/Export pass (it survives Plan calls).
    void SetIkSeed(const SOC_JOINTS& joints) {
        external_ik_seed_ = joints;
        external_ik_config_ = TARGET_CONFIG{};
        has_external_ik_seed_ = true;
    }
    void SetIkSeed(const SOC_JOINTS& joints, const TARGET_CONFIG& config) {
        external_ik_seed_ = joints;
        external_ik_config_ = config;
        has_external_ik_seed_ = true;
    }

 private:
    Config config_;
    double fallback_v_max_ = 0.5;
    double fallback_a_max_ = 2.0;

    MAN_ROBOT robot_param_{};
    FRAME tool_frame_{};
    bool has_kinematic_context_ = false;
    bool ik_recovery_enabled_ = true;
    SOC_JOINTS last_ik_seed_{};
    TARGET_CONFIG last_ik_config_{};
    bool has_ik_seed_ = false;
    SOC_JOINTS external_ik_seed_{};
    TARGET_CONFIG external_ik_config_{};
    bool has_external_ik_seed_ = false;

    // Restart the IK chaining state for a new sampling pass. Re-applies the
    // external seed (if any) so the first solve starts from the seeded branch.
    void ResetIkChain_() {
        has_ik_seed_ = false;
        if (has_external_ik_seed_) {
            last_ik_seed_ = external_ik_seed_;
            last_ik_config_ = external_ik_config_;
            has_ik_seed_ = true;
        }
    }

    // IK recovery: solve joints for a pose-only target via CTarget::CalcJoint.
    // Seeds from the previous sample for branch continuity.
    bool RecoverJoints_(CTarget& target);

    // Compute parametric v/a limits from physical limits and path lengths.
    // parametric_length is the total length of the parameter domain (e.g. 2.0 for Path, 1.0 for PathActionSequence).
    void ComputeParametricFallbackLimits(double physical_length,
                                          double parametric_length,
                                          double* out_param_v_max,
                                          double* out_param_a_max) const;

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
#endif  // ROBOT_TYPE == DUAL_ARM
