#ifndef SERVO_PATH_FLUSH_HPP_
#define SERVO_PATH_FLUSH_HPP_

#include "spline/path_spline.hpp"
#include "servo_step_data.hpp"
#include "robot_transformer.hpp"

namespace servo {

// 把当前路径从 t_current 到路径末端按原时间律走完，fine_point 精确停在
// 段末。t_current 钳制到 [0, path.GetPathTime()]；钳制后已到末端的
// 返回 true、0 步、remaining_time 透传（不重复采样段末）。
// remaining_time_prev/remaining_time_curr 语义同
// SplineInterpolation::Generate，可直接接入余量链。
// 调用前需将 steps[0..max_steps-1] 清零。
bool FlushPathToEnd(const spline::PathSpline& path,
                    double t_current,
                    double sample_time,
                    double remaining_time_prev,
                    const RobotTransformer* transformer,
                    robot_config::RobotSocUnitType unit_type,
                    ServoStepData* steps,
                    int max_steps,
                    int* out_no_of_steps,
                    double* remaining_time_curr);

}  // namespace servo

#endif  // SERVO_PATH_FLUSH_HPP_
