#ifndef DUAL_ARM_SYNCHRONIZER_HPP_
#define DUAL_ARM_SYNCHRONIZER_HPP_
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "trajectory_types.hpp"
#include "dual_arm_dyn.hpp"
#include "dynamic_planner.hpp"
#include "dynamics_config_adapter.hpp"
#include "confbase/dualarm_conf.hpp"
#include <vector>
#include <memory>
#include <string>

namespace action {

class DualArmSynchronizer {
public:
    struct Config {
        // 2026-09-01 flake 根因修复：必须零初始化。
        // 未初始化时 FillTorqueLimits13 的 "already configured" 启发式
        // （sum>1e-6 早退）会把栈垃圾当用户配置保留，导致同步结果依赖
        // 栈残留（间歇 kSyncStretchInfeasible）。
        interpolation::Vec13 tau_max{};
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

    DualArmSynchronizer();
    ~DualArmSynchronizer();

    bool Init(const robot_config::DynamicsConfigAdapter& adapter,
              robot_config::DualArmSocUnitType left_unit,
              robot_config::DualArmSocUnitType right_unit,
              const std::string& base_link,
              const std::string& body_link);

    bool IsInitialized() const { return initialized_; }
    const interpolation::DualArmDyn* GetDualArmDynamics() const {
      return dual_arm_dyn_.get();
    }

    // Main entry: find minimum feasible synchronized makespan
    bool Synchronize(const TrajectoryData& traj_left_in,
                     const TrajectoryData& traj_right_in,
                     const Config& config,
                     Result& out);

private:
    std::unique_ptr<interpolation::DualArmDyn> dual_arm_dyn_;
    bool initialized_ = false;

    bool SampleState_(const TrajectoryData& traj, double t,
                      se3_twist::Vec6& q, se3_twist::Vec6& qd,
                      se3_twist::Vec6& qdd) const;

    bool CheckCoupling_(const TrajectoryData& traj_L,
                        const TrajectoryData& traj_R,
                        double T_target,
                        const interpolation::Vec13& tau_max,
                        int samples,
                        int& worst_joint,
                        double& max_violation_ratio) const;
};

} // namespace action

#endif // DUAL_ARM_SYNCHRONIZER_HPP_
#endif  // ROBOT_TYPE == DUAL_ARM
