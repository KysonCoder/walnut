#ifndef SERVO_PATH_STOP_HPP_
#define SERVO_PATH_STOP_HPP_

#include "spline/path_spline.hpp"
#include "servo_step_data.hpp"
#include "robot_transformer.hpp"
#include "spline_interpolation.hpp"

namespace servo {

// 沿当前 PathSpline 几何三次制动到零速。见
// docs/superpowers/specs/2026-07-27-path-stop-flush-design.md。
class PathStop {
public:
    PathStop();

    bool Initialize(const RobotTransformer* transformer, robot_config::RobotSocUnitType unit_type);

    // 从 path 的 t_current 处（当前速度由 path 原时间律给出）开始，
    // 用三次制动律替换原时间律，沿原几何减速到零速。
    // remaining_time_prev/remaining_time_curr 语义与
    // SplineInterpolation::Generate 相同，直接接入余量链。
    // l_stop（可为 nullptr）：制动剖面末步的归一化路径位置 [0,1]——
    // 正常停稳 = l0+v0·T/3（<1）；恰停段末/越限截断/已在段末 = 1.0；
    // 已静止 = 当前 l0；参数非法 = 不写。
    // 返回 false：参数非法、未初始化、或无法在 max_decel 限制内停在
    // 路径末端之前（此时仍按 max_decel 制动并钳制在 l=1，steps 与
    // 步数有效，调用方可据返回值告警）。
    bool GenerateBraking(const spline::PathSpline& path,
                         double t_current,
                         double max_decel,
                         double sample_time,
                         double remaining_time_prev,
                         ServoStepData* steps,
                         int max_steps,
                         int* out_no_of_steps,
                         double* remaining_time_curr,
                         double* l_stop = nullptr);

    // 从零速停点 l_stop 起，用三次加速律从零速接回 path 原时间律，
    // 覆盖 path 的 [l_stop, 1] 剩余几何。几何不变（不重算路径）；
    // 加速相终点速度 = 原时间律在接合点的速度，接合后与原程序
    // 未暂停时的速度剖面一致。输出格式与 GenerateBraking 一致。
    // l_stop >= 1-eps（段已走完）：*out_no_of_steps = 0 返回 true
    // （空剖面，调用方直接流后续段）。
    // 剩余几何内无接合点（原律末端非零速/加速距离不足）：按 max_accel
    // 加速恰落在 l=1，步数据有效但返回 false（镜像 GenerateBraking
    // 越限语义）。
    // 其余非法输入（l_stop 越界/NaN、max_accel<=0 等）返回 false，0 步。
    bool GenerateResume(const spline::PathSpline& path,
                        double l_stop,
                        double max_accel,
                        double sample_time,
                        double remaining_time_prev,
                        ServoStepData* steps,
                        int max_steps,
                        int* out_no_of_steps,
                        double* remaining_time_curr);

private:
    static constexpr double kEpsilon = 1.0e-9;
    bool initialized_ = false;
    SplineInterpolation interp_;

    // l → t 反查（主样条 l 对 t 单调增，60 次二分）
    static bool InvertL(const spline::PathSpline& path, double l_target,
                        double* t_out);

    // 接合点求解：g(t) = l(t) − l_stop − 2·v(t)²/(3·max_accel) 在
    // [t_stop, path_end] 上的根。返回 false = 防御性失败（求值失败）；
    // *feasible = false = 剩余几何内无接合点（g(path_end) < 0）。
    bool SolveJunction(const spline::PathSpline& path, double l_stop,
                       double max_accel, double t_stop,
                       double* t_junction, double* v_junction,
                       bool* feasible) const;
};

}  // namespace servo

#endif  // SERVO_PATH_STOP_HPP_
