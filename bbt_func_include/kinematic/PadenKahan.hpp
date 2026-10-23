#ifndef PADEN_KAHAN_H
#define PADEN_KAHAN_H

#include "math/quaternion.hpp"
#include "math/se3_twist_ops.hpp"
#include <cmath>
#include <array>
#include <stdexcept>
#include <algorithm>
#include <limits>

/**
 * @brief Paden-Kahan 子问题求解工具类
 * @details 实现机器人学中三种经典的Paden-Kahan子问题求解，以及指数积法构建齐次变换矩阵。
 *          所有接口使用 bbt_include/math 库中的类型，不依赖 Eigen。
 */
class PadenKahan {
public:
    /**
     * @brief 子问题1：绕单一轴旋转到目标位置
     * @param[in] twist_v twist的线速度部分 (v = -w × q)
     * @param[in] twist_w twist的角速度部分 (旋转轴 w)
     * @param[in] pp 起始点
     * @param[in] pk 目标点
     * @return 旋转角度（弧度），使用atan2确保正确象限
     * @note 要求 twist_w 为单位向量（|w| = 1）
     */
    static double subproblem1(const math::Cartesian3<double>& twist_v,
                              const math::Cartesian3<double>& twist_w,
                              const math::Cartesian3<double>& pp,
                              const math::Cartesian3<double>& pk);

    /**
     * @brief 子问题2：绕两个连续轴旋转到目标位置
     * @param[in] v1 第一个twist的线速度部分
     * @param[in] w1 第一个twist的角速度部分
     * @param[in] v2 第二个twist的线速度部分
     * @param[in] w2 第二个twist的角速度部分
     * @param[in] pp 起始点
     * @param[in] pk 目标点
     * @return 2×2 解矩阵，以 se3_twist::Mat4 的前两行两列存储
     *         result[i][j] 对应第 i 组解中第 j 个轴的角度
     * @note 要求 w1, w2 为单位向量
     */
    static se3_twist::Mat4 subproblem2(const math::Cartesian3<double>& v1,
                                       const math::Cartesian3<double>& w1,
                                       const math::Cartesian3<double>& v2,
                                       const math::Cartesian3<double>& w2,
                                       const math::Cartesian3<double>& pp,
                                       const math::Cartesian3<double>& pk);

    /**
     * @brief 子问题3：绕单一轴旋转到特定距离
     * @param[in] twist_v twist的线速度部分
     * @param[in] twist_w twist的角速度部分
     * @param[in] pp 起始点
     * @param[in] pk 目标点
     * @param[in] de 目标距离
     * @return 包含两个解的数组 {theta_1, theta_2}
     * @note 要求 twist_w 为单位向量
     */
    static std::array<double, 2> subproblem3(const math::Cartesian3<double>& twist_v,
                                             const math::Cartesian3<double>& twist_w,
                                             const math::Cartesian3<double>& pp,
                                             const math::Cartesian3<double>& pk,
                                             double de);

    /**
     * @brief 指数积法构建齐次变换矩阵
     * @param[in] w 旋转轴（单位向量）
     * @param[in] q 旋转轴上的一点
     * @param[in] theta 旋转角度（弧度）
     * @return 4×4 齐次变换矩阵
     * @note 使用 se3_twist::Exp 计算矩阵指数
     */
    static se3_twist::Mat4 twistPOE(const math::Cartesian3<double>& w,
                                    const math::Cartesian3<double>& q,
                                    double theta);

    /**
     * @brief 计算向量的斜对称矩阵
     * @param[in] v 三维向量
     * @return 对应的3×3斜对称矩阵
     */
    static se3_twist::Mat3 skewSymmetric(const math::Cartesian3<double>& v);

private:
    /**
     * @brief 解3×3线性方程组 A*x = b
     * @param[in] A 3×3系数矩阵
     * @param[in] b 右侧向量
     * @param[out] x 解向量
     * @return 成功返回true，矩阵奇异返回false
     */
    static bool solve3x3(const se3_twist::Mat3& A,
                         const math::Cartesian3<double>& b,
                         double x[3]);

    static constexpr double PI = 3.14159265358979323846;
    static constexpr double EPSILON = 1e-6;
};


// ============================================================================
// 斜对称矩阵生成
// ============================================================================
inline se3_twist::Mat3 PadenKahan::skewSymmetric(const math::Cartesian3<double>& v) {
    return {{
        {{  0.0, -v.z,  v.y}},
        {{  v.z,  0.0, -v.x}},
        {{-v.y,  v.x,  0.0}}
    }};
}

// ============================================================================
// 3×3线性方程组求解（高斯消元带部分主元）
// ============================================================================
inline bool PadenKahan::solve3x3(const se3_twist::Mat3& A,
                                 const math::Cartesian3<double>& b,
                                 double x[3]) {
    // 构造增广矩阵
    double a[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = A[i][j];
        }
    }
    a[0][3] = b.x;
    a[1][3] = b.y;
    a[2][3] = b.z;

    // 高斯消元（部分主元）
    for (int col = 0; col < 3; ++col) {
        // 找主元
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) {
                pivot = row;
            }
        }
        if (std::fabs(a[pivot][col]) < EPSILON) {
            return false;
        }
        // 交换行
        if (pivot != col) {
            for (int j = col; j < 4; ++j) {
                std::swap(a[col][j], a[pivot][j]);
            }
        }
        // 消去下方元素
        for (int row = col + 1; row < 3; ++row) {
            double factor = a[row][col] / a[col][col];
            for (int j = col; j < 4; ++j) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }

    // 回代
    x[2] = a[2][3] / a[2][2];
    x[1] = (a[1][3] - a[1][2] * x[2]) / a[1][1];
    x[0] = (a[0][3] - a[0][1] * x[1] - a[0][2] * x[2]) / a[0][0];
    return true;
}

// ============================================================================
// 子问题1：绕单轴旋转到目标位置
// ============================================================================
inline double PadenKahan::subproblem1(const math::Cartesian3<double>& twist_v,
                                      const math::Cartesian3<double>& twist_w,
                                      const math::Cartesian3<double>& pp,
                                      const math::Cartesian3<double>& pk) {
    // r1 = w × v / |w|²  —— 旋转轴上离原点最近的点
    double w_sq = twist_w.Dot(twist_w);
    if (w_sq < EPSILON) {
        throw std::invalid_argument("旋转轴w的模长必须大于零");
    }
    math::Cartesian3<double> r1 = twist_w.Cross(twist_v) * (1.0 / w_sq);

    // 计算点到旋转轴的投影（在垂直于w的平面上）
    math::Cartesian3<double> u = pp - r1;
    double w_dot_u = twist_w.Dot(u);
    math::Cartesian3<double> up = u - twist_w * (w_dot_u / w_sq);

    math::Cartesian3<double> v_vec = pk - r1;
    double w_dot_v = twist_w.Dot(v_vec);
    math::Cartesian3<double> vp = v_vec - twist_w * (w_dot_v / w_sq);

    // 使用atan2确保正确象限
    return std::atan2(twist_w.Dot(up.Cross(vp)), up.Dot(vp));
}

// ============================================================================
// 子问题2：绕双轴旋转到目标位置
// ============================================================================
inline se3_twist::Mat4 PadenKahan::subproblem2(const math::Cartesian3<double>& v1,
                                               const math::Cartesian3<double>& w1,
                                               const math::Cartesian3<double>& v2,
                                               const math::Cartesian3<double>& w2,
                                               const math::Cartesian3<double>& pp,
                                               const math::Cartesian3<double>& pk) {
    se3_twist::Mat4 result = se3_twist::utils::mat4_zero();

    double w1_sq = w1.Dot(w1);
    double w2_sq = w2.Dot(w2);
    if (w1_sq < EPSILON || w2_sq < EPSILON) {
        return result;
    }

    // 各旋转轴上离原点最近的点
    math::Cartesian3<double> r1 = w1.Cross(v1) * (1.0 / w1_sq);
    math::Cartesian3<double> r2 = w2.Cross(v2) * (1.0 / w2_sq);

    // 两旋转轴交点
    math::Cartesian3<double> pr;
    math::Cartesian3<double> w1_cross_w2 = w1.Cross(w2);
    if (w1_cross_w2.Length() > EPSILON) {
        // 轴线不平行时求交点
        // A = [w1, -w2, w1×w2]
        se3_twist::Mat3 A = {{
            {{ w1.x, -w2.x, w1_cross_w2.x}},
            {{ w1.y, -w2.y, w1_cross_w2.y}},
            {{ w1.z, -w2.z, w1_cross_w2.z}}
        }};
        math::Cartesian3<double> b = r2 - r1;
        double x[3];
        if (!solve3x3(A, b, x)) {
            return result;
        }
        pr = r1 + w1 * x[0];
    } else {
        return result;
    }

    // 检查交点有效性
    if (!std::isfinite(pr.x) || !std::isfinite(pr.y) || !std::isfinite(pr.z)) {
        return result;
    }

    // 准备计算参数
    math::Cartesian3<double> u = pp - pr;
    math::Cartesian3<double> v = pk - pr;
    double w1w2 = w1.Dot(w2);
    double denominator = w1w2 * w1w2 - 1.0;

    // 检查分母有效性
    if (std::fabs(denominator) < EPSILON) {
        return result;
    }

    // 计算中间参数
    double a = (w1w2 * w2.Dot(u) - w1.Dot(v)) / denominator;
    double b_val = (w1w2 * w1.Dot(v) - w2.Dot(u)) / denominator;
    double Cw1w2_sq = w1_cross_w2.Dot(w1_cross_w2);
    double g2 = (u.Dot(u) - a * a - b_val * b_val - 2.0 * a * b_val * w1w2) / Cw1w2_sq;

    // 检查实数解存在性
    if (g2 < 0.0) {
        return result;
    }

    // 计算两个可能解
    double g = std::sqrt(g2);
    math::Cartesian3<double> pc = pr + w1 * a + w2 * b_val + w1_cross_w2 * g;
    math::Cartesian3<double> pd = pr + w1 * a + w2 * b_val - w1_cross_w2 * g;

    // 准备投影计算
    math::Cartesian3<double> m = pc - pr;
    math::Cartesian3<double> n = pd - pr;
    math::Cartesian3<double> up = u - w2 * (w2.Dot(u) / w2_sq);
    math::Cartesian3<double> vp = v - w1 * (w1.Dot(v) / w1_sq);

    // 计算四个旋转角度
    math::Cartesian3<double> mp = m - w1 * (w1.Dot(m) / w1_sq);
    math::Cartesian3<double> np = n - w1 * (w1.Dot(n) / w1_sq);
    math::Cartesian3<double> up2 = u - w2 * (w2.Dot(u) / w2_sq);
    math::Cartesian3<double> m_w2 = m - w2 * (w2.Dot(m) / w2_sq);
    math::Cartesian3<double> n_w2 = n - w2 * (w2.Dot(n) / w2_sq);

    double t11 = std::atan2(w1.Dot(mp.Cross(vp)), mp.Dot(vp));
    double t21 = std::atan2(w2.Dot(up2.Cross(m_w2)), up2.Dot(m_w2));
    double t12 = std::atan2(w1.Dot(np.Cross(vp)), np.Dot(vp));
    double t22 = std::atan2(w2.Dot(up2.Cross(n_w2)), up2.Dot(n_w2));

    result[0][0] = t11; result[0][1] = t21;
    result[1][0] = t12; result[1][1] = t22;
    return result;
}

// ============================================================================
// 子问题3：绕单一轴旋转到特定距离
// ============================================================================
inline std::array<double, 2> PadenKahan::subproblem3(const math::Cartesian3<double>& twist_v,
                                                      const math::Cartesian3<double>& twist_w,
                                                      const math::Cartesian3<double>& pp,
                                                      const math::Cartesian3<double>& pk,
                                                      double de) {
    double w_sq = twist_w.Dot(twist_w);
    if (w_sq < EPSILON) {
        throw std::invalid_argument("旋转轴w的模长必须大于零");
    }

    math::Cartesian3<double> r1 = twist_w.Cross(twist_v) * (1.0 / w_sq);

    // 计算投影向量
    math::Cartesian3<double> u = pp - r1;
    math::Cartesian3<double> up = u - twist_w * (twist_w.Dot(u) / w_sq);
    double nup = up.Length();

    math::Cartesian3<double> v = pk - r1;
    math::Cartesian3<double> vp = v - twist_w * (twist_w.Dot(v) / w_sq);
    double nvp = vp.Length();

    // 计算基础角度
    double alfa1 = std::atan2(twist_w.Dot(up.Cross(vp)), up.Dot(vp));

    // 计算距离相关参数
    double dep2 = de * de - std::pow(twist_w.Dot(pp - pk), 2);

    if (nup < EPSILON || nvp < EPSILON) {
        return {alfa1, alfa1};
    }

    double beta = (nup * nup + nvp * nvp - dep2) / (2.0 * nup * nvp);

    // 处理数值误差
    beta = std::max(-1.0, std::min(1.0, beta));

    // 计算并返回两个解
    double beta1 = std::acos(beta);
    return {alfa1 - beta1, alfa1 + beta1};
}

// ============================================================================
// 指数积法实现
// ============================================================================
inline se3_twist::Mat4 PadenKahan::twistPOE(const math::Cartesian3<double>& w,
                                            const math::Cartesian3<double>& q,
                                            double theta) {
    // 构建 twist: ξ = [w; v] where v = -(w × q) = q × w
    math::Cartesian3<double> wxq = w.Cross(q);
    se3_twist::Vec3 w_arr = {w.x, w.y, w.z};
    se3_twist::Vec3 v_arr = {-wxq.x, -wxq.y, -wxq.z};

    se3_twist::Twist xi;
    xi.omega() = w_arr;
    xi.vel() = v_arr;

    // 计算 exp(ξ * theta)
    se3_twist::Twist xi_scaled = se3_twist::Scale()(theta, xi);
    return se3_twist::Exp()(xi_scaled);
}

#endif // PADEN_KAHAN_H
