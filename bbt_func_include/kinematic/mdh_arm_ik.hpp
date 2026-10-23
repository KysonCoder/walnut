/**
 * @file mdh_arm_ik.hpp
 * @brief NieNieDual 单臂逆运动学 (AASW: PG7+PK2+PK1)
 *
 * 算法链 (来自 AASWIK.m / ST24R):
 *   Step1: PG7(PardosGotorSeven) → 4组 (θ1,θ2,θ3)
 *   Step2: PK2(PadenKahanTwo)     → 每组×2 → 8组 (θ4,θ5)
 *   Step3: PK1(PadenKahanOne)     → 每组×1 → 8组 (θ6)
 *
 * 4层筛选:
 *   第0层: 奇异点规避 (可操作度)
 *   第1层: 正解验证 (位姿误差 < 阈值)
 *   第2层: 关节限位约束
 *   第3层: 最短行程 (距离参考关节最近)
 *
 * 依赖:
 *   PadenKahan.hpp (PK1/PK2/PK3/twistPOE)
 *   mdh_arm_kin.hpp (FK用于验证)
 *   se3_twist_ops.hpp (Exp/Log/mat4运算)
 *
 * @date 2026-06-16
 */

#ifndef MDH_ARM_IK_HPP_
#define MDH_ARM_IK_HPP_

#include "nie_arm_conf.hpp"
#include "mdh_arm_kin.hpp"
#include "PadenKahan.hpp"
#include "math/se3_twist_ops.hpp"
#include "math/quaternion.hpp"

#include <array>
#include <cmath>
#include <cfloat>

// ── 6-DOF 单臂 screw axes (在 arm基座系 下) ─────────────────
// 对应 AASWIK.m: w=[0 1 0;1 0 0;1 0 0;0 1 0;1 0 0;0 1 0]
//              ap(:,i) 在 Y轴[0,yi,0] 上
struct NieArmScrewAxes {
    // 关节轴上一点 (3×6), mm
    math::Cartesian3<double> p[6];
    // 关节转轴方向 (3×6), 单位向量
    math::Cartesian3<double> w[6];
    // twist: [vx,vy,vz,wx,wy,wz]; v = -w × p
    se3_twist::Twist xi[6];

    // 零位末端位姿 Hst0 (4×4)
    se3_twist::Mat4 Hst0;

    NieArmScrewAxes() {
        // ap = [0,       0,          0,            0,            0,            0;
        //       0,      L1,     L1+L2,    L1+L2+L3,    L1+L2+L3,    L1+L2+L3;
        //       0,       0,          0,            0,            0,            0]
        double p_y[6] = {0.0, NIE_L1, NIE_L1+NIE_L2,
                          NIE_L1+NIE_L2+NIE_L3,
                          NIE_L1+NIE_L2+NIE_L3,
                          NIE_L1+NIE_L2+NIE_L3};

        // w = [0 1 0; 1 0 0; 1 0 0; 0 1 0; 1 0 0; 0 1 0]
        double wx[6] = {0, 1, 1, 0, 1, 0};
        double wy[6] = {1, 0, 0, 1, 0, 1};
        double wz[6] = {0, 0, 0, 0, 0, 0};

        for (int i = 0; i < 6; ++i) {
            p[i] = math::Cartesian3<double>(0.0, p_y[i], 0.0);
            w[i] = math::Cartesian3<double>(wx[i], wy[i], wz[i]);
            // v = -w × p
            math::Cartesian3<double> v = w[i].Cross(p[i]);
            v = math::Cartesian3<double>(-v.x, -v.y, -v.z);
            xi[i].omega() = {w[i].x, w[i].y, w[i].z};
            xi[i].vel()  = {v.x, v.y, v.z};
        }

        // Hst0 = 零位末端位姿 (用MDH FK计算)
        MdhArmParams arm_params;
        for (int i = 0; i < 6; ++i) {
            arm_params.links[i].alpha = NIE_ALPHA[i];
            arm_params.links[i].a     = NIE_A[i];
            arm_params.links[i].d     = NIE_D[i];
            arm_params.links[i].theta_offset = NIE_THETA_OFFSET[i];
        }
        arm_params.tool_d = NIE_LT;
        double zero[6] = {0};
        MdhFkResult fk0 = MdhForwardKinematics(arm_params, zero);
        Hst0 = fk0.frames[6];  // T0_tool at home
    }
};

// ── exp(ξ̂·θ) : 使用 se3_twist::Exp ──────────────────────
inline se3_twist::Mat4 ExpScrew(const se3_twist::Twist& xi, double theta) {
    se3_twist::Twist xit = se3_twist::Scale()(theta, xi);
    return se3_twist::Exp()(xit);
}

// ── PG3: PardosGotorThree ───────────────────────────────
// 平移 screw x1=[v;0;0;0] 移动点 pp 使 pp到pk的距离=de
// 返回 2 个位移解 [t1; t2]
inline std::array<double, 2> PG3(const se3_twist::Twist& x1,
                                  const math::Cartesian3<double>& pp,
                                  const math::Cartesian3<double>& pk,
                                  double de) {
    const auto& v1 = x1.vel();  // 前3分量 = v
    math::Cartesian3<double> v1c(v1[0], v1[1], v1[2]);
    math::Cartesian3<double> kmp(pk.x - pp.x, pk.y - pp.y, pk.z - pp.z);
    double kmpp = v1c.Dot(kmp);
    double root = kmpp * kmpp - kmp.Dot(kmp) + de * de;
    double t1 = kmpp, t2 = kmpp;
    if (root > 0.0) {
        double sr = std::sqrt(root);
        t1 += sr;
        t2 -= sr;
    }
    return {t1, t2};
}

// ── PG4: PardosGotorFour ────────────────────────────────
// 两个平行旋转 screw x1,x2 移动 pp→pk, 返回 2×2 [θ1;θ2]
inline std::array<std::array<double, 2>, 2>
PG4(const se3_twist::Twist& x1, const se3_twist::Twist& x2,
    const math::Cartesian3<double>& pp,
    const math::Cartesian3<double>& pk) {
    const auto& v1 = x1.vel();  const auto& w1v = x1.omega();
    const auto& v2 = x2.vel();  const auto& w2v = x2.omega();
    math::Cartesian3<double> w1c(w1v[0], w1v[1], w1v[2]);
    math::Cartesian3<double> w2c(w2v[0], w2v[1], w2v[2]);

    // r = w×v / |w|²  (旋转轴上最近点)
    math::Cartesian3<double> r1 = w1c.Cross(
        math::Cartesian3<double>(v1[0],v1[1],v1[2]))
        * (1.0 / w1c.Dot(w1c));
    math::Cartesian3<double> r2 = w2c.Cross(
        math::Cartesian3<double>(v2[0],v2[1],v2[2]))
        * (1.0 / w2c.Dot(w2c));

    // v = pk - r1 投影到⊥w1
    math::Cartesian3<double> vv(pk.x - r1.x, pk.y - r1.y, pk.z - r1.z);
    double vw1 = w1c.Dot(vv);
    math::Cartesian3<double> vp(vv.x - w1c.x*vw1, vv.y - w1c.y*vw1, vv.z - w1c.z*vw1);
    double nvp = vp.Length();
    math::Cartesian3<double> c1(r1.x + w1c.x*vw1, r1.y + w1c.y*vw1, r1.z + w1c.z*vw1);

    // u = pp - r2
    math::Cartesian3<double> uu(pp.x - r2.x, pp.y - r2.y, pp.z - r2.z);
    double uw2 = w2c.Dot(uu);
    math::Cartesian3<double> up(uu.x - w2c.x*uw2, uu.y - w2c.y*uw2, uu.z - w2c.z*uw2);
    double nup = up.Length();
    math::Cartesian3<double> c2(r2.x + w2c.x*uw2, r2.y + w2c.y*uw2, r2.z + w2c.z*uw2);

    // 两圆交点 pc, pd
    math::Cartesian3<double> c2c1(c2.x - c1.x, c2.y - c1.y, c2.z - c1.z);
    double nc2c1 = c2c1.Length();
    math::Cartesian3<double> wa(c2c1.x/nc2c1, c2c1.y/nc2c1, c2c1.z/nc2c1);
    math::Cartesian3<double> wh = w1c.Cross(wa);

    math::Cartesian3<double> pc, pd;
    if (nc2c1 >= (nvp + nup) || nvp >= (nc2c1 + nup) || nup >= (nc2c1 + nvp)) {
        pc = math::Cartesian3<double>(c1.x + nvp*wa.x, c1.y + nvp*wa.y, c1.z + nvp*wa.z);
        pd = pc;
    } else {
        double a = (nc2c1*nc2c1 - nup*nup + nvp*nvp) / (2.0 * nc2c1);
        double h = std::sqrt(std::abs(nvp*nvp - a*a));
        pc = math::Cartesian3<double>(c1.x + a*wa.x + h*wh.x,
                                       c1.y + a*wa.y + h*wh.y,
                                       c1.z + a*wa.z + h*wh.z);
        pd = math::Cartesian3<double>(c1.x + a*wa.x - h*wh.x,
                                       c1.y + a*wa.y - h*wh.y,
                                       c1.z + a*wa.z - h*wh.z);
    }

    // PK1 解 θ
    auto pk1 = [&](const math::Cartesian3<double>& pc_,
                    const math::Cartesian3<double>& r,
                    const math::Cartesian3<double>& w,
                    const math::Cartesian3<double>& vp_ref) -> double {
        math::Cartesian3<double> m(pc_.x - r.x, pc_.y - r.y, pc_.z - r.z);
        double mw = w.Dot(m);
        math::Cartesian3<double> mp(m.x - w.x*mw, m.y - w.y*mw, m.z - w.z*mw);
        return std::atan2(w.Dot(mp.Cross(vp_ref)), mp.Dot(vp_ref));
    };

    double t11 = pk1(pc, r1, w1c, vp);
    double t12 = pk1(pd, r1, w1c, vp);
    // MATLAB: atan2(w2·(up × m2p), up·m2p)
    // C++ pk1: atan2(w2·(m2p × up), m2p·up) = -MATLAB → negate
    double t21 = -pk1(pc, r2, w2c, up);
    double t22 = -pk1(pd, r2, w2c, up);

    return {{{t11, t21}, {t12, t22}}};
}

// ── PG7: PardosGotorSeven ──────────────────────────────
// 1个skew+2个平行旋转: exp(E1^θ1)*exp(E2^θ2)*exp(E3^θ3)*pp = pk
// E2||E3 (平行), E1与它们不平行
// 返回 4×3 [θ1,θ2,θ3] 解矩阵
inline std::array<std::array<double, 3>, 4>
PG7(const se3_twist::Twist& x1, const se3_twist::Twist& x2,
    const se3_twist::Twist& x3,
    const math::Cartesian3<double>& pp,
    const math::Cartesian3<double>& pk) {
    const auto& v1 = x1.vel();  const auto& w1v = x1.omega();
    const auto& v3 = x3.vel();  const auto& w3v = x3.omega();
    math::Cartesian3<double> w1c(w1v[0], w1v[1], w1v[2]);
    math::Cartesian3<double> w3c(w3v[0], w3v[1], w3v[2]);

    math::Cartesian3<double> r1 = w1c.Cross(
        math::Cartesian3<double>(v1[0],v1[1],v1[2])) * (1.0 / w1c.Dot(w1c));
    math::Cartesian3<double> r3 = w3c.Cross(
        math::Cartesian3<double>(v3[0],v3[1],v3[2])) * (1.0 / w3c.Dot(w3c));

    // v = pk-r1 投影
    math::Cartesian3<double> vv(pk.x - r1.x, pk.y - r1.y, pk.z - r1.z);
    double vw1 = w1c.Dot(vv);
    math::Cartesian3<double> vp1(vv.x - w1c.x*vw1, vv.y - w1c.y*vw1, vv.z - w1c.z*vw1);
    double nvp = vp1.Length();
    math::Cartesian3<double> o1(r1.x + w1c.x*vw1, r1.y + w1c.y*vw1, r1.z + w1c.z*vw1);

    // u = pp-r3 投影
    math::Cartesian3<double> uu(pp.x - r3.x, pp.y - r3.y, pp.z - r3.z);
    double uw3 = w3c.Dot(uu);
    math::Cartesian3<double> up3(uu.x - w3c.x*uw3, uu.y - w3c.y*uw3, uu.z - w3c.z*uw3);
    double nup = up3.Length();
    math::Cartesian3<double> o3(r3.x + w3c.x*uw3, r3.y + w3c.y*uw3, r3.z + w3c.z*uw3);

    // 两平面交线
    double d1 = w1c.Dot(o1);
    double d3 = w3c.Dot(o3);
    math::Cartesian3<double> v4 = w1c.Cross(w3c);  // 交线方向
    double w13 = w1c.Dot(w3c);
    double den = 1.0 - w13 * w13;
    math::Cartesian3<double> r4;
    if (std::abs(den) > 1e-12) {
        r4 = math::Cartesian3<double>(
            (w1c.x*(d1 - d3*w13) + w3c.x*(d3 - d1*w13)) / den,
            (w1c.y*(d1 - d3*w13) + w3c.y*(d3 - d1*w13)) / den,
            (w1c.z*(d1 - d3*w13) + w3c.z*(d3 - d1*w13)) / den);
    } else {
        r4 = o1;  // 退化
    }

    // PG3: 交线×圆
    se3_twist::Twist x4;
    x4.omega() = {0.0, 0.0, 0.0};  // 平移screw: w=0
    x4.vel()  = {v4.x, v4.y, v4.z};
    auto t4 = PG3(x4, r4, o1, nvp);

    // 两个交点 pc, pd
    math::Cartesian3<double> pc(r4.x + t4[0]*v4.x, r4.y + t4[0]*v4.y, r4.z + t4[0]*v4.z);
    math::Cartesian3<double> pd(r4.x + t4[1]*v4.x, r4.y + t4[1]*v4.y, r4.z + t4[1]*v4.z);

    // PK1 求 θ₁
    auto pk1c = [&](const math::Cartesian3<double>& pc_,
                     const math::Cartesian3<double>& r,
                     const math::Cartesian3<double>& w,
                     const math::Cartesian3<double>& vp) -> double {
        math::Cartesian3<double> m(pc_.x - r.x, pc_.y - r.y, pc_.z - r.z);
        double mw = w.Dot(m);
        math::Cartesian3<double> mp(m.x - w.x*mw, m.y - w.y*mw, m.z - w.z*mw);
        return std::atan2(w.Dot(mp.Cross(vp)), mp.Dot(vp));
    };

    double t1ck = pk1c(pc, r1, w1c, vp1);
    double t1dk = pk1c(pd, r1, w1c, vp1);

    // PG4 求 θ₂,θ₃
    auto t23c = PG4(x2, x3, pp, pc);
    auto t23d = PG4(x2, x3, pp, pd);

    return {{
        {{t1ck, t23c[0][0], t23c[0][1]}},  // t1ck, t2ec, t3pe
        {{t1ck, t23c[1][0], t23c[1][1]}},  // t1ck, t2fc, t3pf
        {{t1dk, t23d[0][0], t23d[0][1]}},  // t1dk, t2ed, t3pe
        {{t1dk, t23d[1][0], t23d[1][1]}}   // t1dk, t2fd, t3pf
    }};
}

// ── 筛选结果 ──────────────────────────────────────────
struct IkResult {
    double joints[6];        // 关节角 (rad) — 第一组解
    double pose_error;       // 位姿误差 (mm+rad)
    bool   valid;            // 是否有有效解
    int    num_passed;       // 通过筛选的解的个数
    double all_joints[8][6]; // 所有通过筛选的解
};

// ── 单臂逆运动学 (AASW: PG7+PK2+PK1, 8组解) ────────────
// Hstd: 目标位姿 (4×4, 臂基座坐标系下)
// ref_joints[6]: 参考关节角 (Type2: 用于最短行程, Type1: 传nullptr用零位)
// joint_min/max[6]: 关节限位 (rad)
// 返回最优解
inline IkResult ArmIK_AASW(const se3_twist::Mat4& Hstd,
                            const double* ref_joints,
                            const double joint_min[6],
                            const double joint_max[6]) {
    IkResult best;
    best.valid = false;
    best.pose_error = DBL_MAX;
    best.pose_error = DBL_MAX;
    for (int i = 0; i < 6; ++i) best.joints[i] = 0.0;

    // ── 获取 screw axes ──
    static const NieArmScrewAxes axes;  // 静态, 只初始化一次

    const NieArmScrewAxes& ax = axes;
    const se3_twist::Mat4& Hst0 = ax.Hst0;

    // Hst0_inv
    se3_twist::Mat4 Hst0_inv = se3_twist::utils::mat4_inverse(Hst0);

    // 参考关节 (默认零位)
    // 构造 FK 参数用于验证
    MdhArmParams arm_params;
    for (int i = 0; i < 6; ++i) {
        arm_params.links[i].alpha = NIE_ALPHA[i];
        arm_params.links[i].a     = NIE_A[i];
        arm_params.links[i].d     = NIE_D[i];
        arm_params.links[i].theta_offset = NIE_THETA_OFFSET[i];
    }
    arm_params.tool_d = NIE_LT;

    // ── Step 1: PG7 求 θ1,θ2,θ3 (4组) ──
    // pf = wrist center = ap(:,4) = [0, L1+L2+L3, 0]
    math::Cartesian3<double> pf(0.0, NIE_L1+NIE_L2+NIE_L3, 0.0);
    // pfk = Hstd * Hst0^{-1} * pf
    se3_twist::Mat4 Hstd_Hst0inv = se3_twist::utils::mat4_mul(Hstd, Hst0_inv);
    double pfx = Hstd_Hst0inv[0][0]*pf.x + Hstd_Hst0inv[0][1]*pf.y
               + Hstd_Hst0inv[0][2]*pf.z + Hstd_Hst0inv[0][3];
    double pfy = Hstd_Hst0inv[1][0]*pf.x + Hstd_Hst0inv[1][1]*pf.y
               + Hstd_Hst0inv[1][2]*pf.z + Hstd_Hst0inv[1][3];
    double pfz = Hstd_Hst0inv[2][0]*pf.x + Hstd_Hst0inv[2][1]*pf.y
               + Hstd_Hst0inv[2][2]*pf.z + Hstd_Hst0inv[2][3];
    math::Cartesian3<double> pfk(pfx, pfy, pfz);

    auto pg7_sol = PG7(ax.xi[0], ax.xi[1], ax.xi[2], pf, pfk);

    // 临时存储 8 组 6-DOF 解
    double all_solutions[8][6];
    int num_valid = 0;

    for (int sol = 0; sol < 4 && num_valid < 8; ++sol) {
        double t1 = pg7_sol[sol][0];
        double t2 = pg7_sol[sol][1];
        double t3 = pg7_sol[sol][2];

        // ── Step 2: PK2 求 θ4,θ5 (2组) ──
        // pp = Hst0的平移列 = tool位置 at home
        math::Cartesian3<double> ppp(Hst0[0][3], Hst0[1][3], Hst0[2][3]);

        // ppk = E3⁻¹*E2⁻¹*E1⁻¹ * Hstd * Hst0⁻¹ * pp
        // 用 -t3 * xi[2], -t2 * xi[1], -t1 * xi[0]
        se3_twist::Twist nt3 = se3_twist::Scale()(-t3, ax.xi[2]);
        se3_twist::Twist nt2 = se3_twist::Scale()(-t2, ax.xi[1]);
        se3_twist::Twist nt1 = se3_twist::Scale()(-t1, ax.xi[0]);

        // pk2pt = expm(-xi3*t3) * expm(-xi2*t2) * expm(-xi1*t1) * Hstd * inv(Hst0) * [pp;1]
        {
            // [pp; 1] → tmp
            double pp4[4] = {ppp.x, ppp.y, ppp.z, 1.0};
            double r1[4] = {
                Hstd_Hst0inv[0][0]*pp4[0]+Hstd_Hst0inv[0][1]*pp4[1]+Hstd_Hst0inv[0][2]*pp4[2]+Hstd_Hst0inv[0][3]*pp4[3],
                Hstd_Hst0inv[1][0]*pp4[0]+Hstd_Hst0inv[1][1]*pp4[1]+Hstd_Hst0inv[1][2]*pp4[2]+Hstd_Hst0inv[1][3]*pp4[3],
                Hstd_Hst0inv[2][0]*pp4[0]+Hstd_Hst0inv[2][1]*pp4[1]+Hstd_Hst0inv[2][2]*pp4[2]+Hstd_Hst0inv[2][3]*pp4[3],
                Hstd_Hst0inv[3][0]*pp4[0]+Hstd_Hst0inv[3][1]*pp4[1]+Hstd_Hst0inv[3][2]*pp4[2]+Hstd_Hst0inv[3][3]*pp4[3]
            };
            se3_twist::Mat4 Et3 = se3_twist::Exp()(nt3);
            se3_twist::Mat4 Et2 = se3_twist::Exp()(nt2);
            se3_twist::Mat4 Et1 = se3_twist::Exp()(nt1);
            se3_twist::Mat4 T1 = se3_twist::utils::mat4_mul(Et2, Et1);
            se3_twist::Mat4 T2 = se3_twist::utils::mat4_mul(Et3, T1);
            double pk2[3] = {
                T2[0][0]*r1[0]+T2[0][1]*r1[1]+T2[0][2]*r1[2]+T2[0][3]*r1[3],
                T2[1][0]*r1[0]+T2[1][1]*r1[1]+T2[1][2]*r1[2]+T2[1][3]*r1[3],
                T2[2][0]*r1[0]+T2[2][1]*r1[1]+T2[2][2]*r1[2]+T2[2][3]*r1[3]
            };

            math::Cartesian3<double> pk2p(pk2[0], pk2[1], pk2[2]);

            auto t45 = PadenKahan::subproblem2(
                math::Cartesian3<double>(ax.xi[3].vel()[0],ax.xi[3].vel()[1],ax.xi[3].vel()[2]),
                math::Cartesian3<double>(ax.xi[3].omega()[0],ax.xi[3].omega()[1],ax.xi[3].omega()[2]),
                math::Cartesian3<double>(ax.xi[4].vel()[0],ax.xi[4].vel()[1],ax.xi[4].vel()[2]),
                math::Cartesian3<double>(ax.xi[4].omega()[0],ax.xi[4].omega()[1],ax.xi[4].omega()[2]),
                ppp, pk2p);

            for (int pk2_sol = 0; pk2_sol < 2 && num_valid < 8; ++pk2_sol) {
                double t4 = t45[pk2_sol][0];
                double t5 = t45[pk2_sol][1];

                // ── Step 3: PK1 求 θ6 ──
                math::Cartesian3<double> pr6p(0.1, 0.2, 0.3);
                se3_twist::Twist nt5 = se3_twist::Scale()(-t5, ax.xi[4]);
                se3_twist::Twist nt4 = se3_twist::Scale()(-t4, ax.xi[3]);
                // Eall_inv = E5⁻¹ * E4⁻¹ * E3⁻¹ * E2⁻¹ * E1⁻¹
                se3_twist::Mat4 Eall_inv = se3_twist::Exp()(nt1);
                Eall_inv = se3_twist::utils::mat4_mul(se3_twist::Exp()(nt2), Eall_inv);
                Eall_inv = se3_twist::utils::mat4_mul(se3_twist::Exp()(nt3), Eall_inv);
                Eall_inv = se3_twist::utils::mat4_mul(se3_twist::Exp()(nt4), Eall_inv);
                Eall_inv = se3_twist::utils::mat4_mul(se3_twist::Exp()(nt5), Eall_inv);

                double pr6_h[4] = {pr6p.x, pr6p.y, pr6p.z, 1.0};
                double r6h[4] = {
                    Hstd_Hst0inv[0][0]*pr6_h[0]+Hstd_Hst0inv[0][1]*pr6_h[1]+Hstd_Hst0inv[0][2]*pr6_h[2]+Hstd_Hst0inv[0][3]*pr6_h[3],
                    Hstd_Hst0inv[1][0]*pr6_h[0]+Hstd_Hst0inv[1][1]*pr6_h[1]+Hstd_Hst0inv[1][2]*pr6_h[2]+Hstd_Hst0inv[1][3]*pr6_h[3],
                    Hstd_Hst0inv[2][0]*pr6_h[0]+Hstd_Hst0inv[2][1]*pr6_h[1]+Hstd_Hst0inv[2][2]*pr6_h[2]+Hstd_Hst0inv[2][3]*pr6_h[3],
                    Hstd_Hst0inv[3][0]*pr6_h[0]+Hstd_Hst0inv[3][1]*pr6_h[1]+Hstd_Hst0inv[3][2]*pr6_h[2]+Hstd_Hst0inv[3][3]*pr6_h[3]
                };
                double pr6k[3] = {
                    Eall_inv[0][0]*r6h[0]+Eall_inv[0][1]*r6h[1]+Eall_inv[0][2]*r6h[2]+Eall_inv[0][3]*r6h[3],
                    Eall_inv[1][0]*r6h[0]+Eall_inv[1][1]*r6h[1]+Eall_inv[1][2]*r6h[2]+Eall_inv[1][3]*r6h[3],
                    Eall_inv[2][0]*r6h[0]+Eall_inv[2][1]*r6h[1]+Eall_inv[2][2]*r6h[2]+Eall_inv[2][3]*r6h[3]
                };

                math::Cartesian3<double> pr6k_c(pr6k[0], pr6k[1], pr6k[2]);
                double t6 = PadenKahan::subproblem1(
                    math::Cartesian3<double>(ax.xi[5].vel()[0],ax.xi[5].vel()[1],ax.xi[5].vel()[2]),
                    math::Cartesian3<double>(ax.xi[5].omega()[0],ax.xi[5].omega()[1],ax.xi[5].omega()[2]),
                    pr6p, pr6k_c);

                // 存储候选解
                if (num_valid < 8) {
                    all_solutions[num_valid][0] = t1;
                    all_solutions[num_valid][1] = t2;
                    all_solutions[num_valid][2] = t3;
                    all_solutions[num_valid][3] = t4;
                    all_solutions[num_valid][4] = t5;
                    all_solutions[num_valid][5] = t6;
                    ++num_valid;
                }
            }
        }
    }

    // ── 2层筛选 ──────────────────────────────────────────
    const double FK_POS_THRESHOLD   = 1e-3;  // 位置阈值 (mm)
    const double FK_ORIENT_THRESHOLD = 1e-4;  // 朝向阈值 (rad)
    best.num_passed = 0;

    for (int s = 0; s < num_valid; ++s) {
        double* cand = all_solutions[s];

        // 筛选1: 关节限位
        bool in_limits = true;
        for (int i = 0; i < 6; ++i) {
            if (cand[i] < joint_min[i] || cand[i] > joint_max[i]) {
                in_limits = false;
                break;
            }
        }
        if (!in_limits) continue;

        // 筛选2: 正解验证
        MdhFkResult fk = MdhForwardKinematics(arm_params, cand);
        se3_twist::Mat4 T_fk = fk.frames[6];

        double pos_err = std::sqrt(
            (T_fk[0][3]-Hstd[0][3])*(T_fk[0][3]-Hstd[0][3]) +
            (T_fk[1][3]-Hstd[1][3])*(T_fk[1][3]-Hstd[1][3]) +
            (T_fk[2][3]-Hstd[2][3])*(T_fk[2][3]-Hstd[2][3]));

        math::RotationMatrix3<double> rot_fk, rot_tgt;
        for (int r=0;r<3;++r) for(int c=0;c<3;++c) {
            rot_fk(r,c)=T_fk[r][c]; rot_tgt(r,c)=Hstd[r][c];
        }
        math::Quaternion<double> q_fk=math::Quaternion<double>::FromRotationMatrix(rot_fk);
        math::Quaternion<double> q_tgt=math::Quaternion<double>::FromRotationMatrix(rot_tgt);
        double orient_err = q_fk.Distance(q_tgt);

        if (pos_err > FK_POS_THRESHOLD || orient_err > FK_ORIENT_THRESHOLD) continue;

        // 收集通过筛选的解
        int n = best.num_passed;
        for (int i = 0; i < 6; ++i) best.all_joints[n][i] = cand[i];
        ++best.num_passed;
    }

    // 筛选3: 时序一致性 — 选距ref_joints最近的解
    if (best.num_passed > 0) {
        best.valid = true;
        double ref_zero[6] = {0};
        if (!ref_joints) ref_joints = ref_zero; // 默认零位参考
        int pick = 0;
        double min_dist = 1e100;
        for (int n = 0; n < best.num_passed; ++n) {
            double dist = 0.0;
            for (int i = 0; i < 6; ++i) {
                double d = best.all_joints[n][i] - ref_joints[i];
                dist += d * d;
            }
            // q3正值加权偏好 (q3<0 → 惩罚, 优先肘向前构型)
            double q3 = best.all_joints[n][2];
            if (q3 < 0.0) dist += q3 * q3 * 10.0;
            if (dist < min_dist) { min_dist = dist; pick = n; }
        }
        for (int i = 0; i < 6; ++i) best.joints[i] = best.all_joints[pick][i];
    }
    return best;
}

// ── 返回全部 8 组候选解 (不过滤, 用于调试/验证) ────────────
struct IkCandidate {
    double joints[6];
    double pos_err;
    double orient_err;
    bool in_limits;
};
inline std::array<IkCandidate, 8> ArmIK_AASW_All(
        const se3_twist::Mat4& Hstd) {
    std::array<IkCandidate, 8> cands{};
    int cnt = 0;

    static const NieArmScrewAxes axes;
    const auto& ax = axes;
    se3_twist::Mat4 Hst0_inv = se3_twist::utils::mat4_inverse(ax.Hst0);

    MdhArmParams arm_params;
    for (int i = 0; i < 6; ++i) {
        arm_params.links[i].alpha=NIE_ALPHA[i]; arm_params.links[i].a=NIE_A[i];
        arm_params.links[i].d=NIE_D[i]; arm_params.links[i].theta_offset=NIE_THETA_OFFSET[i];
    }
    arm_params.tool_d = NIE_LT;

    se3_twist::Mat4 Hstd_Hst0inv = se3_twist::utils::mat4_mul(Hstd, Hst0_inv);
    math::Cartesian3<double> pf(0.0, NIE_L1+NIE_L2+NIE_L3, 0.0);
    double pfx = Hstd_Hst0inv[0][0]*pf.x+Hstd_Hst0inv[0][1]*pf.y+Hstd_Hst0inv[0][2]*pf.z+Hstd_Hst0inv[0][3];
    double pfy = Hstd_Hst0inv[1][0]*pf.x+Hstd_Hst0inv[1][1]*pf.y+Hstd_Hst0inv[1][2]*pf.z+Hstd_Hst0inv[1][3];
    double pfz = Hstd_Hst0inv[2][0]*pf.x+Hstd_Hst0inv[2][1]*pf.y+Hstd_Hst0inv[2][2]*pf.z+Hstd_Hst0inv[2][3];
    auto pg7_sol = PG7(ax.xi[0], ax.xi[1], ax.xi[2], pf,
                        math::Cartesian3<double>(pfx,pfy,pfz));

    math::Cartesian3<double> ppp(ax.Hst0[0][3], ax.Hst0[1][3], ax.Hst0[2][3]);
    math::Cartesian3<double> pr6p(0.1, 0.2, 0.3);

    for (int sol = 0; sol < 4 && cnt < 8; ++sol) {
        double t1=pg7_sol[sol][0], t2=pg7_sol[sol][1], t3=pg7_sol[sol][2];
        // PK2
        se3_twist::Mat4 Et1=se3_twist::Exp()(se3_twist::Scale()(-t1,ax.xi[0]));
        se3_twist::Mat4 Et2=se3_twist::Exp()(se3_twist::Scale()(-t2,ax.xi[1]));
        se3_twist::Mat4 Et3=se3_twist::Exp()(se3_twist::Scale()(-t3,ax.xi[2]));
        se3_twist::Mat4 T2 = se3_twist::utils::mat4_mul(Et3,
            se3_twist::utils::mat4_mul(Et2, Et1));
        double r1[4]={Hstd_Hst0inv[0][0]*ppp.x+Hstd_Hst0inv[0][1]*ppp.y+Hstd_Hst0inv[0][2]*ppp.z+Hstd_Hst0inv[0][3],
                       Hstd_Hst0inv[1][0]*ppp.x+Hstd_Hst0inv[1][1]*ppp.y+Hstd_Hst0inv[1][2]*ppp.z+Hstd_Hst0inv[1][3],
                       Hstd_Hst0inv[2][0]*ppp.x+Hstd_Hst0inv[2][1]*ppp.y+Hstd_Hst0inv[2][2]*ppp.z+Hstd_Hst0inv[2][3],1.0};
        double pk2[3]={T2[0][0]*r1[0]+T2[0][1]*r1[1]+T2[0][2]*r1[2]+T2[0][3],
                        T2[1][0]*r1[0]+T2[1][1]*r1[1]+T2[1][2]*r1[2]+T2[1][3],
                        T2[2][0]*r1[0]+T2[2][1]*r1[1]+T2[2][2]*r1[2]+T2[2][3]};

        auto t45 = PadenKahan::subproblem2(
            math::Cartesian3<double>(ax.xi[3].vel()[0],ax.xi[3].vel()[1],ax.xi[3].vel()[2]),
            math::Cartesian3<double>(ax.xi[3].omega()[0],ax.xi[3].omega()[1],ax.xi[3].omega()[2]),
            math::Cartesian3<double>(ax.xi[4].vel()[0],ax.xi[4].vel()[1],ax.xi[4].vel()[2]),
            math::Cartesian3<double>(ax.xi[4].omega()[0],ax.xi[4].omega()[1],ax.xi[4].omega()[2]),
            ppp, math::Cartesian3<double>(pk2[0],pk2[1],pk2[2]));

        for (int ws=0; ws<2 && cnt<8; ++ws) {
            double t4=t45[ws][0], t5=t45[ws][1];
            // PK1
            se3_twist::Mat4 Eall=se3_twist::Exp()(se3_twist::Scale()(-t1,ax.xi[0]));
            Eall=se3_twist::utils::mat4_mul(se3_twist::Exp()(se3_twist::Scale()(-t2,ax.xi[1])),Eall);
            Eall=se3_twist::utils::mat4_mul(se3_twist::Exp()(se3_twist::Scale()(-t3,ax.xi[2])),Eall);
            Eall=se3_twist::utils::mat4_mul(se3_twist::Exp()(se3_twist::Scale()(-t4,ax.xi[3])),Eall);
            Eall=se3_twist::utils::mat4_mul(se3_twist::Exp()(se3_twist::Scale()(-t5,ax.xi[4])),Eall);
            double rh[4]={Hstd_Hst0inv[0][0]*pr6p.x+Hstd_Hst0inv[0][1]*pr6p.y+Hstd_Hst0inv[0][2]*pr6p.z+Hstd_Hst0inv[0][3],
                           Hstd_Hst0inv[1][0]*pr6p.x+Hstd_Hst0inv[1][1]*pr6p.y+Hstd_Hst0inv[1][2]*pr6p.z+Hstd_Hst0inv[1][3],
                           Hstd_Hst0inv[2][0]*pr6p.x+Hstd_Hst0inv[2][1]*pr6p.y+Hstd_Hst0inv[2][2]*pr6p.z+Hstd_Hst0inv[2][3],1.0};
            double pr6k[3]={Eall[0][0]*rh[0]+Eall[0][1]*rh[1]+Eall[0][2]*rh[2]+Eall[0][3],
                             Eall[1][0]*rh[0]+Eall[1][1]*rh[1]+Eall[1][2]*rh[2]+Eall[1][3],
                             Eall[2][0]*rh[0]+Eall[2][1]*rh[1]+Eall[2][2]*rh[2]+Eall[2][3]};
            double t6 = PadenKahan::subproblem1(
                math::Cartesian3<double>(ax.xi[5].vel()[0],ax.xi[5].vel()[1],ax.xi[5].vel()[2]),
                math::Cartesian3<double>(ax.xi[5].omega()[0],ax.xi[5].omega()[1],ax.xi[5].omega()[2]),
                pr6p, math::Cartesian3<double>(pr6k[0],pr6k[1],pr6k[2]));

            double jj[6]={t1,t2,t3,t4,t5,t6};
            MdhFkResult fk = MdhForwardKinematics(arm_params, jj);
            double pe = std::sqrt((fk.frames[6][0][3]-Hstd[0][3])*(fk.frames[6][0][3]-Hstd[0][3])+
                                   (fk.frames[6][1][3]-Hstd[1][3])*(fk.frames[6][1][3]-Hstd[1][3])+
                                   (fk.frames[6][2][3]-Hstd[2][3])*(fk.frames[6][2][3]-Hstd[2][3]));
            math::RotationMatrix3<double> rfk,rtg;
            for(int r=0;r<3;++r) for(int c=0;c<3;++c){rfk(r,c)=fk.frames[6][r][c];rtg(r,c)=Hstd[r][c];}
            double oe=math::Quaternion<double>::FromRotationMatrix(rfk)
                .Distance(math::Quaternion<double>::FromRotationMatrix(rtg));

            cands[cnt].pos_err=pe; cands[cnt].orient_err=oe;
            for(int i=0;i<6;++i) cands[cnt].joints[i]=jj[i];
            ++cnt;
        }
    }
    return cands;
}

#endif  // MDH_ARM_IK_HPP_
