/**
 * @file mdh_arm_kin.hpp
 * @brief 通用 MDH 6-DOF 串联臂 正运动学 & 几何法(矢量积)速度雅可比
 *
 * 不依赖 CKinGeneRbt, 完全基于 Modified DH 参数自建。
 * 几何法 Jacobian:
 *   J_ω_i = z_{i-1}                        (关节轴方向)
 *   J_v_i = z_{i-1} × (p_ee - p_{i-1})     (矢量积, revolute)
 *
 * @date 2026-06-16
 */

#ifndef MDH_ARM_KIN_HPP_
#define MDH_ARM_KIN_HPP_

#include "math/se3_twist_ops.hpp"
#include "math/quaternion.hpp"
#include "math/pose.hpp"

#include <array>
#include <cmath>
#include <cstring>

// ── MDH 单连杆参数 ─────────────────────────────────────
struct MdhLink {
    double alpha;         // 连杆扭转角 (rad)
    double a;             // 连杆长度 (mm)
    double d;             // 连杆偏距 (mm)
    double theta_offset;  // 关节角零位偏移 (rad)
};

// ── MDH 臂完整参数 ─────────────────────────────────────
struct MdhArmParams {
    MdhLink links[6];     // 6个连杆
    double tool_d;        // 工具长度 (mm), 沿末端Z轴
};

// ── 单个 MDH 变换矩阵 ──────────────────────────────────
// T = RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
inline se3_twist::Mat4 MdhTransform(const MdhLink& link, double theta) {
    double a  = link.a;
    double d  = link.d;
    double al = link.alpha;
    double th = theta + link.theta_offset;

    double ca  = std::cos(al), sa = std::sin(al);
    double ct  = std::cos(th), st = std::sin(th);

    se3_twist::Mat4 T = se3_twist::utils::mat4_zero();
    T[0][0] = ct;       T[0][1] = -st;       T[0][2] = 0.0;     T[0][3] = a;
    T[1][0] = ca * st;  T[1][1] = ca * ct;   T[1][2] = -sa;    T[1][3] = -sa * d;
    T[2][0] = sa * st;  T[2][1] = sa * ct;   T[2][2] =  ca;    T[2][3] =  ca * d;
    T[3][0] = 0.0;      T[3][1] = 0.0;       T[3][2] = 0.0;    T[3][3] = 1.0;
    return T;
}

// ── 4×4矩阵乘法 (专用, 略快于泛型) ──────────────────────
inline se3_twist::Mat4 MdhMatMul(const se3_twist::Mat4& A,
                                  const se3_twist::Mat4& B) {
    se3_twist::Mat4 C = se3_twist::utils::mat4_zero();
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 4; ++k)
            for (int j = 0; j < 4; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// ── FK 结果: 各帧矩阵 + 末端信息 ────────────────────────
struct MdhFkResult {
    se3_twist::Mat4 frames[7];   // frames[0..5]: T0_1..T0_6
                                  // frames[6]: T0_tool
    math::Cartesian3<double> ee_pos;    // 末端位置 (mm)
    math::Quaternion<double>  ee_orient; // 末端朝向
};

// ── 正运动学 (FK) ──────────────────────────────────────
// joints[6]: 关节角 (rad), 不含零位偏移(由参数自带)
inline MdhFkResult MdhForwardKinematics(const MdhArmParams& params,
                                         const double joints[6]) {
    MdhFkResult result;
    se3_twist::Mat4 T = se3_twist::utils::mat4_identity();

    for (int i = 0; i < 6; ++i) {
        se3_twist::Mat4 Ti = MdhTransform(params.links[i], joints[i]);
        T = MdhMatMul(T, Ti);
        result.frames[i] = T;  // T0_{i+1}
    }

    // Tool: 固定变换 Tz(Lt)
    MdhLink tool_link = {0.0, 0.0, params.tool_d, 0.0};
    T = MdhMatMul(T, MdhTransform(tool_link, 0.0));
    result.frames[6] = T;

    // 提取位姿
    result.ee_pos = math::Cartesian3<double>(T[0][3], T[1][3], T[2][3]);
    math::RotationMatrix3<double> rot;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            rot(r, c) = T[r][c];
    result.ee_orient = math::Quaternion<double>::FromRotationMatrix(rot);

    return result;
}

// ── 几何法 速度雅可比 (6×6) ────────────────────────────
// 行0-2: 角速度 [ωx, ωy, ωz]
// 行3-5: 线速度 [vx, vy, vz]
// 列0-5: 关节1-6
//
// MDH 关节i的旋转轴 = Z轴 位于 Rx(α_{i-1})*Tx(a_{i-1}) 之后
//   z_i = 第3列 of (T₀_{i-1} * Rx(α_{i-1}) * Tx(a_{i-1}))
//   p_i = 平移 of (T₀_{i-1} * Rx(α_{i-1}) * Tx(a_{i-1}))
//
// 几何法 (矢量积):
//   Jω_i = z_i
//   Jv_i = z_i × (p_ee - p_i)
inline void MdhGeometricJacobian(const MdhArmParams& params,
                                  const double joints[6],
                                  double jacobian[6][6]) {
    // 1. 先算 FK，获得末端位置和各帧累积
    MdhFkResult fk = MdhForwardKinematics(params, joints);
    math::Cartesian3<double> p_ee = fk.ee_pos;

    // 2. 对每个关节, 计算旋转轴z_i和轴上点p_i
    //    z_i = Z轴 of (Tprev * Rx(α)*Tx(a))
    //    p_i = 平移 of (Tprev * Rx(α)*Tx(a))
    se3_twist::Mat4 T_acc = se3_twist::utils::mat4_identity();  // T₀_{i-1}

    for (int i = 0; i < 6; ++i) {
        const MdhLink& link = params.links[i];

        // 构造 Rx(α) * Tx(a)
        double ca = std::cos(link.alpha), sa = std::sin(link.alpha);
        se3_twist::Mat4 T_pre = se3_twist::utils::mat4_identity();
        T_pre[1][1] = ca;   T_pre[1][2] = -sa;
        T_pre[2][1] = sa;   T_pre[2][2] =  ca;
        T_pre[0][3] = link.a;  // Tx(a)

        // T_axis = T_acc * Rx(α) * Tx(a)  → 提取Z轴
        se3_twist::Mat4 T_axis = MdhMatMul(T_acc, T_pre);

        math::Cartesian3<double> z(
            T_axis[0][2], T_axis[1][2], T_axis[2][2]);
        math::Cartesian3<double> p(
            T_axis[0][3], T_axis[1][3], T_axis[2][3]);

        // 角速度列
        jacobian[0][i] = z.x;
        jacobian[1][i] = z.y;
        jacobian[2][i] = z.z;

        // 线速度列 = z × (p_ee - p)
        math::Cartesian3<double> dp(p_ee.x - p.x, p_ee.y - p.y, p_ee.z - p.z);
        math::Cartesian3<double> v = z.Cross(dp);
        jacobian[3][i] = v.x;
        jacobian[4][i] = v.y;
        jacobian[5][i] = v.z;

        // 更新 T_acc = T_acc * Ti (完整变换,包含Rz(θ)*Tz(d))
        T_acc = fk.frames[i];
    }
}

// ── 提取4×4矩阵的Z轴方向 ───────────────────────────────
inline math::Cartesian3<double> MdhGetZAxis(const se3_twist::Mat4& T) {
    return math::Cartesian3<double>(T[0][2], T[1][2], T[2][2]);
}

// ── 可操作度 μ = sqrt(det(J·Jᵀ)) ────────────────────────
inline double Manipulability6(const double J[6][6]) {
    double A[6][6] = {};
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k)
            for (int j = 0; j < 6; ++j)
                A[i][j] += J[i][k] * J[k][j];
    // Gaussian elimination for determinant
    double det = 1.0;
    for (int i = 0; i < 6; ++i) {
        int pivot = i;
        for (int j = i+1; j < 6; ++j)
            if (std::abs(A[j][i]) > std::abs(A[pivot][i])) pivot = j;
        if (std::abs(A[pivot][i]) < 1e-15) return 0.0;
        if (pivot != i) {
            for (int j = i; j < 6; ++j) std::swap(A[i][j], A[pivot][j]);
            det = -det;
        }
        det *= A[i][i];
        for (int j = i+1; j < 6; ++j) {
            double f = A[j][i] / A[i][i];
            for (int k = i; k < 6; ++k) A[j][k] -= f * A[i][k];
        }
    }
    return std::sqrt(std::abs(det));
}

// ── DLS 阻尼逆: qd = Jᵀ(J·Jᵀ + λ²I)⁻¹ · vd ─────────────
// mu = 当前可操作度, mu0 = 阈值, lam_max = 最大阻尼
inline void DLSVelocity(const double J[6][6],
                        const double vd[6],
                        double mu, double mu0, double lam_max,
                        double qd[6]) {
    double lam2 = 0.0;
    if (mu < mu0)
        lam2 = lam_max * lam_max * (1.0 - mu / mu0) * (1.0 - mu / mu0);

    // J*Jᵀ + λ²I → 6×6 对称矩阵
    double M[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k)
                M[i][j] += J[i][k] * J[j][k];
            if (i == j) M[i][j] += lam2;
        }
    }

    // Solve M·x = vd via Gaussian elimination
    double a[6][7];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) a[i][j] = M[i][j];
        a[i][6] = vd[i];
    }
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        for (int r = col+1; r < 6; ++r)
            if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
        if (pivot != col)
            for (int j = col; j < 7; ++j) std::swap(a[col][j], a[pivot][j]);
        double piv = a[col][col];
        if (std::abs(piv) < 1e-15) { for (int i=0;i<6;++i) qd[i]=0.0; return; }
        for (int j = col; j < 7; ++j) a[col][j] /= piv;
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            double f = a[r][col];
            for (int j = col; j < 7; ++j) a[r][j] -= f * a[col][j];
        }
    }
    // Jᵀ · x → qd
    double x[6] = {a[0][6], a[1][6], a[2][6], a[3][6], a[4][6], a[5][6]};
    for (int i = 0; i < 6; ++i) {
        qd[i] = 0.0;
        for (int j = 0; j < 6; ++j)
            qd[i] += J[j][i] * x[j];
    }
}

// ── 速度限幅 ────────────────────────────────────────────
inline void ClampVelocity(double qd[6], const double qd_max[6]) {
    for (int i = 0; i < 6; ++i) {
        if (qd[i] >  qd_max[i]) qd[i] =  qd_max[i];
        if (qd[i] < -qd_max[i]) qd[i] = -qd_max[i];
    }
}

// ── 组合: DLS + 限幅 + 构型选择 (奇异穿越完整方案) ──────────
//   joints[6]:  当前关节角 (rad)
//   vd[6]:      期望末端速度 [ωx,ωy,ωz, vx,vy,vz]
//   qd[6] (出): 关节角速度 (rad/s)
//   qd_max[6]:  关节速度上限
//   mu0:        DLS 激活阈值
//   lam_max:    DLS 最大阻尼
inline void SolveVelocitySingularitySafe(
        const MdhArmParams& params,
        const double joints[6],
        const double vd[6],
        const double qd_max[6],
        double mu0, double lam_max,
        double qd[6]) {
    double J[6][6] = {};
    MdhGeometricJacobian(params, joints, J);
    double mu = Manipulability6(J);
    DLSVelocity(J, vd, mu, mu0, lam_max, qd);
    ClampVelocity(qd, qd_max);
}

// ── 提取4×4矩阵的平移部分 ──────────────────────────────
inline math::Cartesian3<double> MdhGetTranslation(const se3_twist::Mat4& T) {
    return math::Cartesian3<double>(T[0][3], T[1][3], T[2][3]);
}

#endif  // MDH_ARM_KIN_HPP_
