#ifndef ACC_BOUND_CALCULATOR_HPP_
#define ACC_BOUND_CALCULATOR_HPP_

#include "math/se3_twist_ops.hpp"

namespace pathplanning {

using Vec6 = se3_twist::Vec6;

struct AccBounds {
    double s_ddot_min;
    double s_ddot_max;
    bool feasible;
};

struct VelocityLimit {
    double s_dot_max;
    bool feasible;
};

class AccBoundCalculator {
public:
    // Compute s_ddot bounds at given s_dot.
    // Returns true if computation succeeded (check out.feasible for feasibility).
    // m: effective inertia M * dq/ds
    // c: quadratic term coefficient (M * d2q/ds2 + C(q,dq/ds)*dq/ds)
    // g: gravity vector
    // tau_max/tau_min: per-joint torque limits
    // s_dot: current path velocity
    static bool Compute(const Vec6& m, const Vec6& c, const Vec6& g,
                        const Vec6& tau_max, const Vec6& tau_min,
                        double s_dot, AccBounds& out);

    // Compute max feasible s_dot at this path point (velocity limit curve).
    static bool ComputeVelocityLimit(const Vec6& m, const Vec6& c, const Vec6& g,
                                     const Vec6& tau_max, const Vec6& tau_min,
                                     double* s_dot_lim);
};

} // namespace pathplanning

#endif
