#ifndef JOINT_DYNAMICS_ENRICHER_HPP
#define JOINT_DYNAMICS_ENRICHER_HPP
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "math/se3_twist_ops.hpp"
#include "path_action_types.hpp"
#include "GenrArmDyn.hpp"
#include <memory>

namespace action {

// Abstract dynamics backend (extensible to dual-arm)
class IDynamicsBackend {
public:
    virtual ~IDynamicsBackend() = default;
    virtual int Nq() const = 0;
    virtual int ComputeMassDiag(const se3_twist::Vec6& q,
                                 se3_twist::Vec6& mass_diag_out) = 0;
    virtual int ComputeGravity(const se3_twist::Vec6& q,
                                se3_twist::Vec6& torque_g_out) = 0;
    virtual int ComputeCoriolis(const se3_twist::Vec6& q,
                                 const se3_twist::Vec6& qd,
                                 se3_twist::Vec6& torque_c_out) = 0;
};

// Concrete backend wrapping GenrArmDyn (single-arm, 6-DOF)
class GenrArmDynBackend : public IDynamicsBackend {
public:
    explicit GenrArmDynBackend(interpolation::GenrArmDyn& dyn);

    int Nq() const override { return 6; }
    int ComputeMassDiag(const se3_twist::Vec6& q,
                         se3_twist::Vec6& mass_diag_out) override;
    int ComputeGravity(const se3_twist::Vec6& q,
                        se3_twist::Vec6& torque_g_out) override;
    int ComputeCoriolis(const se3_twist::Vec6& q,
                         const se3_twist::Vec6& qd,
                         se3_twist::Vec6& torque_c_out) override;

private:
    interpolation::GenrArmDyn& dyn_;
};

class JointDynamicsEnricher {
public:
    struct Config {
        double micro_interval_dL{0.01};   // fraction of segment param range
    };

    explicit JointDynamicsEnricher(Config cfg,
                                   std::unique_ptr<IDynamicsBackend> backend);

    // Strict: returns false if ANY segment fails enrichment
    bool Enrich(PathActionSequence& sequence) const;

private:
    struct MicroSample {
        double L{0.0};
        se3_twist::Vec6 q{};
    };

    bool SampleAndFitD2q(const PathActionSegment& seg,
                          double dL,
                          se3_twist::Vec6& d2q_dL2_out) const;

    bool EvaluateDynamics(const se3_twist::Vec6& q,
                          const se3_twist::Vec6& qd,
                          se3_twist::Vec6& mass_diag,
                          se3_twist::Vec6& torque_g,
                          se3_twist::Vec6& torque_c) const;

    Config config_;
    std::unique_ptr<IDynamicsBackend> backend_;
};

} // namespace action

#endif
#endif  // ROBOT_TYPE == DUAL_ARM
