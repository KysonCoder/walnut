#ifndef DYNAMIC_PLANNER_HPP_
#define DYNAMIC_PLANNER_HPP_

#include "acc_bound_calculator.hpp"
#include "GenrArmDyn.hpp"
#include "drivechain/drive_chain_calculator.hpp"
#include "drivechain/drive_chain_tables.hpp"
#include <vector>

namespace pathplanning {

struct CartesianAccBounds {
    Vec6 a_min;
    Vec6 a_max;
    bool feasible;
};

class DynamicPlanner {
 public:
    struct Config {
        int32_t unit_type;
        drivechain::DriveChainMode drivechain_mode;
        drivechain::EmergencyPhase emergency_phase;

        Config()
            : unit_type(0),  // RIGHT_ARM in DUAL_ARM layout
              drivechain_mode(drivechain::DriveChainMode::kNormal),
              emergency_phase(drivechain::EmergencyPhase::kPhase1) {}
    };

    DynamicPlanner() = default;
    virtual ~DynamicPlanner() = default;

    DynamicPlanner(interpolation::GenrArmDyn* dynamics, const Config& config);

    // Test-only injection. If drivechain_data is non-null, it is used directly
    // instead of querying RobotConfigStore.
    DynamicPlanner(interpolation::GenrArmDyn* dynamics,
                   const Config& config,
                   const drivechain::DriveChainUnitData* drivechain_data);

    bool ComputeAccBound(const Vec6& q, const Vec6& dq_ds,
                         const Vec6& d2q_ds2, double s_dot,
                         AccBounds& bounds);

    bool ComputeVelocityLimit(const Vec6& q, const Vec6& dq_ds,
                              const Vec6& d2q_ds2, VelocityLimit& limit);

    bool ComputeCartesianAccBound(const Vec6& q, const Vec6& q_dot,
                                  const Vec6& dq_ds, const Vec6& d2q_ds2,
                                  double s_dot, CartesianAccBounds& bounds);

    bool SampleVelocityLimitCurve(const std::vector<Vec6>& q_path,
                                  const std::vector<Vec6>& dq_ds_path,
                                  const std::vector<Vec6>& d2q_ds2_path,
                                  std::vector<double>& s_dot_lim_out);

    // Returns lower/upper bounds on path acceleration d²L/dt² at path position
    // L with path velocity dL/dt. Default implementation returns false;
    // override for a concrete backend.
    virtual bool ComputeTorqueBounds(double path_pos, double path_vel,
                                     double& lower_acc_out,
                                     double& upper_acc_out);

    bool IsValid() const { return is_valid_; }

    // Test-only accessor for private ComputeTorqueLimits_.
    bool TestComputeTorqueLimits(const Vec6& dq_ds, double s_dot,
                                 Vec6& tau_max_out, Vec6& tau_min_out) const {
        return ComputeTorqueLimits_(dq_ds, s_dot, tau_max_out, tau_min_out);
    }

 private:
    interpolation::GenrArmDyn* dynamics_ = nullptr;
    Config config_;
    drivechain::DriveChainCalculator drivechain_calc_;
    const drivechain::DriveChainUnitData* drivechain_data_ = nullptr;
    bool is_valid_ = false;

    bool ComputeDynamicsTerms_(const Vec6& q, const Vec6& dq_ds,
                               const Vec6& d2q_ds2,
                               Vec6& m_out, Vec6& c_out, Vec6& g_out) const;

    bool ComputeTorqueLimits_(const Vec6& dq_ds, double s_dot,
                              Vec6& tau_max_out, Vec6& tau_min_out) const;
};

} // namespace pathplanning

#endif
