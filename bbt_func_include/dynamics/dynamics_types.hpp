// func/dynamics/include/dynamics_types.hpp
#ifndef FUNC_DYNAMICS_INCLUDE_DYNAMICS_TYPES_HPP_
#define FUNC_DYNAMICS_INCLUDE_DYNAMICS_TYPES_HPP_

#include <array>

namespace interpolation {

// DOF limits are chosen to match confbase::kMaxUnitJoints (7) and
// confbase::kMaxNoOfJoints (57). They are hard-coded here so that this header
// stays free of confbase/robot.hpp and can be included alongside the confbase
// dynamics adapter header.
constexpr int kMaxUnitDof = 7;
constexpr int kMaxRobotDof = 57;

using VecA = std::array<double, kMaxUnitDof>;
using VecR = std::array<double, kMaxRobotDof>;

template <int Rows, int Cols>
using Mat = std::array<std::array<double, Cols>, Rows>;

using MatAxA = Mat<kMaxUnitDof, kMaxUnitDof>;
using Mat6xA = Mat<6, kMaxUnitDof>;

} // namespace interpolation

#endif // FUNC_DYNAMICS_INCLUDE_DYNAMICS_TYPES_HPP_
