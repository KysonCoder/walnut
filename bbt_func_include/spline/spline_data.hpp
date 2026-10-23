#ifndef SPLINE_SPLINE_DATA_HPP_
#define SPLINE_SPLINE_DATA_HPP_

#include <array>
#include "confbase/robot.hpp"

namespace spline {

struct SplineData {
    int order = 0;
    double delta_x = 0.0;
    double c[4] = {0.0, 0.0, 0.0, 0.0};
    bool valueError = false;
    bool isempty = true;
};

struct JointSplineData {
    double delta_x = 0.0;
    SplineData splines[robot_config::kRobotMaxJoints];
    std::array<bool, robot_config::kRobotMaxJoints> joint_mask{};
};

struct PathSplineData {
    SplineData main_spline_data;        // l(t), t in [0, path_time], l in [0, 1]
    double path_time = 0.0;             // total trajectory time
    double main_length = 0.0;           // physical standard path length (meters)
    double l_length[robot_config::kRobotMaxUnits];  // per-SOC physical path length (meters)
    JointSplineData joint_spline_data;  // q(l), l in [0, 1]
};

} // namespace spline

#endif
