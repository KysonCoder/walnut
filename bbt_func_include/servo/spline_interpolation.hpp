#ifndef SERVO_SPLINE_INTERPOLATION_HPP_
#define SERVO_SPLINE_INTERPOLATION_HPP_

#include "spline/path_spline.hpp"
#include "servo_step_data.hpp"
#include "robot_transformer.hpp"

namespace servo {

class SplineInterpolation {
public:
    SplineInterpolation();

    bool Initialize(const RobotTransformer* transformer, robot_config::RobotSocUnitType unit_type);

    // Caller must zero steps[0..max_steps-1] before calling.
    //
    // 采样时刻由余量驱动的全局网格决定：
    //   t_k = t_start + (k+1)*sample_time - remaining_time_prev
    // remaining_time_prev 为上一段/上一窗口末端积累的余量，取值
    // [0, sample_time]，越界返回 false。首段传 sample_time 可使首采样
    // 恰好落在 t_start；传 0 表示上一段恰好结束在网格点上。
    //
    // fine_point 为 true 且余量 > eps 时，在 t_end 处追加一个精确采样
    // 并将余量清零（用于队列中最后一条 PathSpline）。
    //
    // *remaining_time_curr 输出以 t_end 为基准的余量，可直接作为下一次
    // 调用（其 t_start == 本次 t_end）的 remaining_time_prev。
    bool Generate(const spline::PathSpline& path,
                  double t_start,
                  double t_end,
                  double sample_time,
                  double remaining_time_prev,
                  bool fine_point,
                  ServoStepData* steps,
                  int max_steps,
                  int* out_no_of_steps,
                  double* remaining_time_curr);

private:
    static constexpr double kEpsilon = 1.0e-9;

    const RobotTransformer* transformer_ = nullptr;
    robot_config::RobotSocUnitType unit_type_ = robot_config::RobotSocUnitType::NO_UNIT;
};

}  // namespace servo

#endif
