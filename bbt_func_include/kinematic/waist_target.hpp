/**
 * @file waist_target.hpp
 * @brief NieNieDual 第1部分: 腰部 1-DOF (Z轴旋转 + 抬升L0)
 *
 * FK:   T_waist = Rz(theta) * Tz(L0)
 * Jac:  6×1 列 = [0, 0, 1, -p.y, p.x, 0]^T
 *       其中 p = 被影响臂的末端位置 (由上层传入)
 *
 * @date 2026-06-16
 */

#ifndef WAIST_TARGET_HPP_
#define WAIST_TARGET_HPP_

#include "nie_arm_conf.hpp"
#include "math/se3_twist_ops.hpp"
#include "math/quaternion.hpp"

#include <cmath>

// ── 腰部 FK ────────────────────────────────────────────
// 返回 T = Rz(theta) * Tz(L0)
//       ┌ cosθ  -sinθ  0   0    ┐
//       │ sinθ   cosθ  0   0    │
//       │  0      0    1  L0    │
//       └  0      0    0   1    ┘
inline se3_twist::Mat4 WaistCalcPose(double theta) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    se3_twist::Mat4 T = se3_twist::utils::mat4_zero();
    T[0][0] = c;   T[0][1] = -s;  T[0][2] = 0.0;  T[0][3] = 0.0;
    T[1][0] = s;   T[1][1] =  c;  T[1][2] = 0.0;  T[1][3] = 0.0;
    T[2][0] = 0.0; T[2][1] = 0.0; T[2][2] = 1.0;  T[2][3] = NIE_L0;
    T[3][0] = 0.0; T[3][1] = 0.0; T[3][2] = 0.0;  T[3][3] = 1.0;
    return T;
}

// ── 腰部对臂末端的 Jacobian 贡献 ───────────────────────
// ee_pos: 该臂末端在世界坐标系下的位置 (用于计算 v = ω × p)
// jac_col[6]: 输出 6×1 列向量 [ωx, ωy, ωz, vx, vy, vz]^T
//
// 腰部绕世界Z轴旋转:
//   ω = [0, 0, 1]^T
//   v = ω × ee_pos = [-ee_pos.y, ee_pos.x, 0]^T
inline void WaistJacobianColumn(const math::Cartesian3<double>& ee_pos,
                                 double jac_col[6]) {
    jac_col[0] = 0.0;            // ωx = 0
    jac_col[1] = 0.0;            // ωy = 0
    jac_col[2] = 1.0;            // ωz = 1
    jac_col[3] = -ee_pos.y;      // vx = -p.y
    jac_col[4] =  ee_pos.x;      // vy =  p.x
    jac_col[5] = 0.0;            // vz = 0
}

// ── 腰部逆变换 ─────────────────────────────────────────
// T_waist_inv = Tz(-L0) * Rz(-theta)
inline se3_twist::Mat4 WaistInverse(double theta) {
    double c = std::cos(-theta);
    double s = std::sin(-theta);
    se3_twist::Mat4 T = se3_twist::utils::mat4_zero();
    T[0][0] = c;   T[0][1] = -s;  T[0][2] = 0.0;  T[0][3] = 0.0;
    T[1][0] = s;   T[1][1] =  c;  T[1][2] = 0.0;  T[1][3] = 0.0;
    T[2][0] = 0.0; T[2][1] = 0.0; T[2][2] = 1.0;  T[2][3] = -NIE_L0;
    T[3][0] = 0.0; T[3][1] = 0.0; T[3][2] = 0.0;  T[3][3] = 1.0;
    return T;
}

#endif  // WAIST_TARGET_HPP_
