#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include <array>
#include <cmath>
#include <cstdio>

namespace math {

/**
 * @class RotationMatrix3
 * @brief 3x3 旋转矩阵模板类
 * @tparam T 元素类型，通常为 float 或 double
 */
template <typename T>
class RotationMatrix3 {
 public:
  /** @brief 3x3 矩阵数据，以 std::array<std::array<T, 3>, 3> 形式存储 */
  std::array<std::array<T, 3>, 3> data{};

  /**
   * @brief 访问指定行列的元素（可修改）
   * @param[in] row 行索引（0~2）
   * @param[in] col 列索引（0~2）
   * @return 对应位置元素的引用
   */
  T& operator()(size_t row, size_t col) { return data[row][col]; }

  /**
   * @brief 访问指定行列的元素（只读）
   * @param[in] row 行索引（0~2）
   * @param[in] col 列索引（0~2）
   * @return 对应位置元素的常引用
   */
  const T& operator()(size_t row, size_t col) const { return data[row][col]; }

  /**
   * @brief 返回该旋转矩阵的转置
   * @return 转置后的 RotationMatrix3<T>
   * @details 对于正交旋转矩阵，转置即为其逆矩阵
   */
  RotationMatrix3 Transpose() const {
    RotationMatrix3 m;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        m(i, j) = data[j][i];
      }
    }
    return m;
  }

  /**
   * @brief 旋转矩阵乘法
   * @param[in] other 另一个旋转矩阵
   * @return 乘积结果
   */
  RotationMatrix3 operator*(const RotationMatrix3& other) const {
    RotationMatrix3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        r(i, j) = data[i][0] * other(0, j)
                + data[i][1] * other(1, j)
                + data[i][2] * other(2, j);
    return r;
  }
};

/**
 * @class Cartesian3
 * @brief 三维笛卡尔向量模板类
 * @tparam T 元素类型，通常为 float 或 double
 */
template <typename T>
class Cartesian3 {
 public:
  T x = 0;  /**< @brief X 分量 */
  T y = 0;  /**< @brief Y 分量 */
  T z = 0;  /**< @brief Z 分量 */

  /** @brief 默认构造函数，初始化为零向量 */
  Cartesian3() = default;

  /**
   * @brief 带参数构造函数
   * @param[in] x_val X 分量初始值
   * @param[in] y_val Y 分量初始值
   * @param[in] z_val Z 分量初始值
   */
  Cartesian3(T x_val, T y_val, T z_val) : x(x_val), y(y_val), z(z_val) {}

  /**
   * @brief 计算向量的欧几里得长度
   * @return 向量长度 sqrt(x^2 + y^2 + z^2)
   */
  T Length() const { return std::sqrt(x * x + y * y + z * z); }

  /**
   * @brief 判断两个向量是否相等
   * @param[in] other 另一个 Cartesian3 向量
   * @return 相等返回 true，否则返回 false
   */
  bool operator==(const Cartesian3& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  /**
   * @brief 判断两个向量是否不相等
   * @param[in] other 另一个 Cartesian3 向量
   * @return 不相等返回 true，否则返回 false
   */
  bool operator!=(const Cartesian3& other) const {
    return !(*this == other);
  }

  /**
   * @brief 向量加法
   * @param[in] other 另一个向量
   * @return 相加结果
   */
  Cartesian3 operator+(const Cartesian3& other) const {
    return Cartesian3(x + other.x, y + other.y, z + other.z);
  }

  /**
   * @brief 向量减法
   * @param[in] other 另一个向量
   * @return 相减结果
   */
  Cartesian3 operator-(const Cartesian3& other) const {
    return Cartesian3(x - other.x, y - other.y, z - other.z);
  }

  /**
   * @brief 一元取负运算符
   * @return 各分量取反后的向量
   */
  Cartesian3 operator-() const {
    return Cartesian3(-x, -y, -z);
  }

  /**
   * @brief 向量与标量相乘
   * @param[in] scalar 标量
   * @return 缩放后的向量
   */
  Cartesian3 operator*(T scalar) const {
    return Cartesian3(x * scalar, y * scalar, z * scalar);
  }

  /**
   * @brief 计算点积
   * @param[in] other 另一个向量
   * @return 点积结果
   */
  T Dot(const Cartesian3& other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  /**
   * @brief 计算叉积
   * @param[in] other 另一个向量
   * @return 叉积结果
   */
  Cartesian3 Cross(const Cartesian3& other) const {
    return Cartesian3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x);
  }
};

/**
 * @brief 标量左乘向量
 * @param[in] scalar 标量
 * @param[in] v 向量
 * @return 缩放后的向量
 */
template <typename T>
Cartesian3<T> operator*(T scalar, const Cartesian3<T>& v) {
  return v * scalar;
}

/**
 * @brief 旋转矩阵与向量相乘
 * @param[in] m 3x3 旋转矩阵
 * @param[in] v 三维向量
 * @return 矩阵作用后的向量
 */
template <typename T>
Cartesian3<T> operator*(const RotationMatrix3<T>& m, const Cartesian3<T>& v) {
  return Cartesian3<T>(
      m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
      m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
      m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z);
}

/**
 * @class Quaternion
 * @brief 四元数模板类，采用 {q0, q1, q2, q3} 表示法
 * @details q0 为标量部分，(q1, q2, q3) 为向量部分
 * @tparam T 元素类型，默认为 double
 */
template <typename T = double>
class Quaternion {
 public:
  T q0 = 1;  /**< @brief 标量部分（实部） */
  T q1 = 0;  /**< @brief 向量部分 X */
  T q2 = 0;  /**< @brief 向量部分 Y */
  T q3 = 0;  /**< @brief 向量部分 Z */

  /** @brief 默认构造函数，构造为单位四元数 (1, 0, 0, 0) */
  Quaternion() = default;

  /**
   * @brief 带参数构造函数
   * @param[in] q0_val 标量部分初始值
   * @param[in] q1_val 向量 X 分量初始值
   * @param[in] q2_val 向量 Y 分量初始值
   * @param[in] q3_val 向量 Z 分量初始值
   */
  Quaternion(T q0_val, T q1_val, T q2_val, T q3_val)
      : q0(q0_val), q1(q1_val), q2(q2_val), q3(q3_val) {}

  /**
   * @brief 构造单位四元数
   * @return 单位四元数 Quaternion(1, 0, 0, 0)
   */
  static Quaternion Unity() { return Quaternion(1, 0, 0, 0); }

  /**
   * @brief 构造零四元数
   * @return 零四元数 Quaternion(0, 0, 0, 0)
   */
  static Quaternion Zero() { return Quaternion(0, 0, 0, 0); }

  /**
   * @brief 下标访问运算符（可修改）
   * @param[in] idx 索引 0~3
   * @return 对应分量的引用；若越界则记录 ERROR 日志并回退返回 q3 的引用
   */
  T& operator[](size_t idx) {
    switch (idx) {
      case 0: return q0;
      case 1: return q1;
      case 2: return q2;
      case 3: return q3;
      default:
        std::fprintf(stderr, "[ERROR] Quaternion index out of range: %zu\n", idx);
        return q3;
    }
  }

  /**
   * @brief 下标访问运算符（只读）
   * @param[in] idx 索引 0~3
   * @return 对应分量的常引用；若越界则记录 ERROR 日志并回退返回 q3 的常引用
   */
  const T& operator[](size_t idx) const {
    switch (idx) {
      case 0: return q0;
      case 1: return q1;
      case 2: return q2;
      case 3: return q3;
      default:
        std::fprintf(stderr, "[ERROR] Quaternion index out of range: %zu\n", idx);
        return q3;
    }
  }

  /**
   * @brief 计算四元数的模（2-范数）
   * @return 四元数的模 sqrt(q0^2 + q1^2 + q2^2 + q3^2)
   */
  T Norm() const {
    return std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  }

  /**
   * @brief 将四元数归一化为单位四元数
   * @return 归一化成功返回 true；若模过小则记录 WARN 日志并返回 false
   */
  bool Normalize() {
    T norm = Norm();
    constexpr T kEps = static_cast<T>(1e-5);
    if (norm > kEps) {
      q0 /= norm;
      q1 /= norm;
      q2 /= norm;
      q3 /= norm;
      return true;
    }
    std::fprintf(stderr, "[WARN] Quaternion norm is too small (%.9f), normalization skipped\n", norm);
    return false;
  }

  /**
   * @brief 求共轭四元数
   * @return 共轭四元数 (q0, -q1, -q2, -q3)
   */
  Quaternion Conjugate() const {
    return Quaternion(q0, -q1, -q2, -q3);
  }

  /**
   * @brief 求四元数的逆
   * @return 逆四元数；若为零四元数则记录 ERROR 日志并回退返回单位四元数
   */
  Quaternion Inverse() const {
    T norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (norm_sq == 0) {
      std::fprintf(stderr, "[ERROR] Cannot invert zero quaternion, returning unity\n");
      return Unity();
    }
    T inv_norm_sq = static_cast<T>(1) / norm_sq;
    return Quaternion(q0 * inv_norm_sq, -q1 * inv_norm_sq,
                      -q2 * inv_norm_sq, -q3 * inv_norm_sq);
  }

  /**
   * @brief 强制标量部分为正
   * @details 若 q0 < 0，则整体取反，保证四元数位于同一半球
   */
  void EnforcePositiveScalar() {
    if (q0 < 0) {
      q0 = -q0;
      q1 = -q1;
      q2 = -q2;
      q3 = -q3;
    }
  }

  /**
   * @brief 检查四元数是否已归一化
   * @param[in] tolerance 容差阈值，默认为 1e-5
   * @return |Norm() - 1| <= tolerance 时返回 true
   */
  bool IsNormalized(T tolerance = static_cast<T>(1e-5)) const {
    T norm_err = std::abs(Norm() - static_cast<T>(1));
    return norm_err <= tolerance;
  }

  /**
   * @brief 使用当前四元数旋转一个三维向量
   * @param[in] pos 待旋转的三维向量
   * @return 旋转后的三维向量
   */
  Cartesian3<T> Rotate(const Cartesian3<T>& pos) const {
    Quaternion q_pos(0, pos.x, pos.y, pos.z);
    Quaternion q_inv = Inverse();
    Quaternion q1 = *this * q_pos;
    Quaternion qr = q1 * q_inv;
    return Cartesian3<T>(qr.q1, qr.q2, qr.q3);
  }

  /**
   * @brief 获取旋转后的 X 轴单位向量
   * @return 旋转后坐标系的 X 轴方向
   */
  Cartesian3<T> GetXAxis() const {
    return Rotate(Cartesian3<T>(1, 0, 0));
  }

  /**
   * @brief 获取旋转后的 Y 轴单位向量
   * @return 旋转后坐标系的 Y 轴方向
   */
  Cartesian3<T> GetYAxis() const {
    return Rotate(Cartesian3<T>(0, 1, 0));
  }

  /**
   * @brief 获取旋转后的 Z 轴单位向量
   * @return 旋转后坐标系的 Z 轴方向
   */
  Cartesian3<T> GetZAxis() const {
    return Rotate(Cartesian3<T>(0, 0, 1));
  }

  /**
   * @brief 计算两个四元数之间的距离（测地线角度）
   * @param[in] other 另一个四元数
   * @return 两四元数表示旋转之间的角度距离（弧度）
   */
  T Distance(const Quaternion& other) const {
    Quaternion q_inv = other.Inverse();
    Quaternion diff = *this * q_inv;
    diff.EnforcePositiveScalar();
    T diff_norm = diff.Norm();
    if (diff_norm > 0) {
      diff.q0 /= diff_norm;
    } else {
      std::fprintf(stderr, "[WARN] Distance calculation encountered zero norm, returning 0\n");
      return static_cast<T>(0);
    }
    T clamped = std::max(static_cast<T>(-1), std::min(static_cast<T>(1), diff.q0));
    T dlo = static_cast<T>(2) * std::acos(clamped);
    if (dlo > static_cast<T>(M_PI)) {
      dlo = std::abs(dlo - static_cast<T>(2) * static_cast<T>(M_PI));
    }
    return dlo;
  }

  /**
   * @brief 将四元数转换为 3x3 旋转矩阵
   * @return 对应的 RotationMatrix3<T>
   */
  RotationMatrix3<T> ToRotationMatrix() const {
    RotationMatrix3<T> m;
    T xs = q1 * static_cast<T>(2);
    T ys = q2 * static_cast<T>(2);
    T zs = q3 * static_cast<T>(2);

    T wx = q0 * xs;
    T wy = q0 * ys;
    T wz = q0 * zs;

    T xx = q1 * xs;
    T xy = q1 * ys;
    T xz = q1 * zs;

    T yy = q2 * ys;
    T yz = q2 * zs;
    T zz = q3 * zs;

    m(0, 0) = static_cast<T>(1) - (yy + zz);
    m(0, 1) = xy - wz;
    m(0, 2) = xz + wy;

    m(1, 0) = xy + wz;
    m(1, 1) = static_cast<T>(1) - (xx + zz);
    m(1, 2) = yz - wx;

    m(2, 0) = xz - wy;
    m(2, 1) = yz + wx;
    m(2, 2) = static_cast<T>(1) - (xx + yy);

    return m;
  }

  /**
   * @brief 由 3x3 旋转矩阵构造四元数
   * @param[in] m 3x3 旋转矩阵
   * @return 对应的四元数；若计算结果模为零则记录 ERROR 日志并回退返回单位四元数
   */
  static Quaternion FromRotationMatrix(const RotationMatrix3<T>& m) {
    T trace = m(0, 0) + m(1, 1) + m(2, 2);
    T root;
    T u0, u1, u2, u3;

    if (trace > 0) {
      root = std::sqrt(trace + static_cast<T>(1));
      u0 = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      u1 = (m(2, 1) - m(1, 2)) * root;
      u2 = (m(0, 2) - m(2, 0)) * root;
      u3 = (m(1, 0) - m(0, 1)) * root;
    } else if (m(0, 0) >= m(1, 1) && m(0, 0) >= m(2, 2)) {
      root = std::sqrt(static_cast<T>(1) + m(0, 0) - (m(1, 1) + m(2, 2)));
      u1 = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      u2 = (m(0, 1) + m(1, 0)) * root;
      u3 = (m(2, 0) + m(0, 2)) * root;
      u0 = (m(2, 1) - m(1, 2)) * root;
    } else if (m(1, 1) >= m(2, 2)) {
      root = std::sqrt(static_cast<T>(1) + m(1, 1) - (m(2, 2) + m(0, 0)));
      u2 = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      u3 = (m(1, 2) + m(2, 1)) * root;
      u1 = (m(0, 1) + m(1, 0)) * root;
      u0 = (m(0, 2) - m(2, 0)) * root;
    } else {
      root = std::sqrt(static_cast<T>(1) + m(2, 2) - (m(0, 0) + m(1, 1)));
      u3 = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      u1 = (m(2, 0) + m(0, 2)) * root;
      u2 = (m(1, 2) + m(2, 1)) * root;
      u0 = (m(1, 0) - m(0, 1)) * root;
    }

    T norm = std::sqrt(u0 * u0 + u1 * u1 + u2 * u2 + u3 * u3);
    if (norm == 0) {
      std::fprintf(stderr, "[ERROR] Rotation matrix to quaternion produced zero norm, returning unity\n");
      return Unity();
    }
    u0 /= norm; u1 /= norm; u2 /= norm; u3 /= norm;

    if (u0 < 0) {
      u0 = -u0; u1 = -u1; u2 = -u2; u3 = -u3;
    }

    return Quaternion(u0, u1, u2, u3);
  }

  /**
   * @brief 由轴角表示构造四元数
   * @param[in] angle 旋转角度（弧度）
   * @param[in] axis 旋转轴（三维向量）
   * @return 对应的单位四元数
   */
  static Quaternion FromAxisAngle(T angle, const Cartesian3<T>& axis) {
    T sin_half = std::sin(angle / static_cast<T>(2));
    T cos_half = std::cos(angle / static_cast<T>(2));
    return Quaternion(cos_half, sin_half * axis.x,
                      sin_half * axis.y, sin_half * axis.z);
  }

  /**
   * @brief 将四元数转换为轴角表示
   * @param[out] angle 输出的旋转角度（弧度）
   * @param[out] axis 输出的旋转轴
   * @details 对于极小旋转会使用小角度近似，并在必要时记录 WARN / INFO / DEBUG 日志
   */
  void ToAxisAngle(T* angle, Cartesian3<T>* axis) const {
    Quaternion local_q = *this;
    bool offset = false;

    if (std::abs(local_q.q0) > static_cast<T>(1)) {
      std::fprintf(stderr, "[WARN] Quaternion scalar part |q0| > 1 (%.9f), normalizing\n", local_q.q0);
      local_q.Normalize();
    }

    if (local_q.q0 < 0) {
      offset = true;
      local_q.q0 = -local_q.q0;
    }

    *angle = static_cast<T>(2) * std::acos(local_q.q0);

    constexpr T kEps10 = static_cast<T>(1e-10);
    if (std::abs(*angle) > static_cast<T>(2.5e-2)) {
      T sin_half = std::sin(*angle / static_cast<T>(2));
      if (std::abs(sin_half) < kEps10) {
        std::fprintf(stderr, "[WARN] sin(angle/2) too small, axis may be unstable\n");
      }
      axis->x = local_q.q1 / sin_half;
      axis->y = local_q.q2 / sin_half;
      axis->z = local_q.q3 / sin_half;
    } else {
      T vec_len = local_q.q1 * local_q.q1 +
                  local_q.q2 * local_q.q2 +
                  local_q.q3 * local_q.q3;
      if (vec_len < kEps10) {
        std::fprintf(stderr, "[INFO] Very small rotation, axis set to zero\n");
        *angle = 0;
        *axis = Cartesian3<T>(0, 0, 0);
      } else {
        std::fprintf(stderr, "[DEBUG] Using small-angle approximation for axis-angle\n");
        T sin_theta_approx_half = std::sqrt(vec_len);
        *angle = sin_theta_approx_half * static_cast<T>(2);
        axis->x = local_q.q1 / sin_theta_approx_half;
        axis->y = local_q.q2 / sin_theta_approx_half;
        axis->z = local_q.q3 / sin_theta_approx_half;
      }
    }

    if (offset) {
      *angle = static_cast<T>(2) * static_cast<T>(M_PI) - *angle;
    }
  }

  /**
   * @brief 由 Euler ZYX 角构造四元数
   * @param[in] z 绕 Z 轴旋转角（弧度）
   * @param[in] y 绕 Y 轴旋转角（弧度）
   * @param[in] x 绕 X 轴旋转角（弧度）
   * @return 对应的单位四元数
   */
  static Quaternion FromEulerZyx(T z, T y, T x) {
    T ca = std::cos(z), sa = std::sin(z);
    T cb = std::cos(y), sb = std::sin(y);
    T cc = std::cos(x), sc = std::sin(x);

    RotationMatrix3<T> m;
    m(0, 0) = ca * cb;
    m(0, 1) = ca * sb * sc - sa * cc;
    m(0, 2) = ca * sb * cc + sa * sc;

    m(1, 0) = sa * cb;
    m(1, 1) = sa * sb * sc + ca * cc;
    m(1, 2) = sa * sb * cc - ca * sc;

    m(2, 0) = -sb;
    m(2, 1) = cb * sc;
    m(2, 2) = cb * cc;

    return FromRotationMatrix(m);
  }

  /**
   * @brief 将四元数转换为 Euler ZYX 角
   * @param[out] z 输出的绕 Z 轴旋转角（弧度）
   * @param[out] y 输出的绕 Y 轴旋转角（弧度）
   * @param[out] x 输出的绕 X 轴旋转角（弧度）
   * @details 在 Y = +/- 90 deg 的退化情况下会记录 DEBUG 日志
   */
  void ToEulerZyx(T* z, T* y, T* x) const {
    RotationMatrix3<T> m = ToRotationMatrix();
    T sqr = std::sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0));
    *y = static_cast<T>(std::atan2(-m(2, 0), sqr));

    constexpr T kEps4 = static_cast<T>(1e-4);
    T pi_half = static_cast<T>(M_PI) * static_cast<T>(0.5);

    if (std::abs(*y - pi_half) <= kEps4) {
      std::fprintf(stderr, "[DEBUG] Euler ZYX degenerate at +90 deg Y\n");
      *z = 0;
      *x = static_cast<T>(std::atan2(m(0, 1), m(1, 1)));
    } else if (std::abs(*y + pi_half) <= kEps4) {
      std::fprintf(stderr, "[DEBUG] Euler ZYX degenerate at -90 deg Y\n");
      *z = 0;
      *x = -static_cast<T>(std::atan2(m(0, 1), m(1, 1)));
    } else {
      *z = static_cast<T>(std::atan2(m(1, 0), m(0, 0)));
      *x = static_cast<T>(std::atan2(m(2, 1), m(2, 2)));
    }
  }

  /**
   * @brief 由 RPY（Roll-Pitch-Yaw）角构造四元数
   * @param[in] r Roll（绕 X 轴）
   * @param[in] p Pitch（绕 Y 轴）
   * @param[in] y Yaw（绕 Z 轴）
   * @return 对应的单位四元数
   */
  static Quaternion FromRpy(T r, T p, T y) {
    T ca = std::cos(y), sa = std::sin(y);
    T cb = std::cos(p), sb = std::sin(p);
    T cy = std::cos(r), sy = std::sin(r);

    RotationMatrix3<T> m;
    m(0, 0) = ca * cb;
    m(0, 1) = ca * sb * sy - sa * cy;
    m(0, 2) = ca * sb * cy + sa * sy;

    m(1, 0) = sa * cb;
    m(1, 1) = sa * sb * sy + ca * cy;
    m(1, 2) = sa * sb * cy - ca * sy;

    m(2, 0) = -sb;
    m(2, 1) = cb * sy;
    m(2, 2) = cb * cy;

    return FromRotationMatrix(m);
  }

  /**
   * @brief 由固定轴 ZYX 旋转角构造四元数
   * @param[in] Z 绕固定 Z 轴角（弧度）
   * @param[in] Y 绕固定 Y 轴角（弧度）
   * @param[in] X 绕固定 X 轴角（弧度）
   * @return 对应的单位四元数
   */
  static Quaternion FromFixedZyx(T Z, T Y, T X) {
    RotationMatrix3<T> m;
    m(0, 0) = std::cos(Y) * std::cos(Z);
    m(0, 1) = -std::cos(Y) * std::sin(Z);
    m(0, 2) = std::sin(Y);

    m(1, 0) = std::sin(X) * std::sin(Y) * std::cos(Z) + std::cos(X) * std::sin(Z);
    m(1, 1) = -std::sin(X) * std::sin(Y) * std::sin(Z) + std::cos(X) * std::cos(Z);
    m(1, 2) = -std::sin(X) * std::cos(Y);

    m(2, 0) = -std::cos(X) * std::sin(Y) * std::cos(Z) + std::sin(X) * std::sin(Z);
    m(2, 1) = std::cos(X) * std::sin(Y) * std::sin(Z) + std::sin(X) * std::cos(Z);
    m(2, 2) = std::cos(X) * std::cos(Y);

    return FromRotationMatrix(m);
  }

  /**
   * @brief 将四元数转换为固定轴 ZYX 旋转角
   * @param[out] Z 输出的绕固定 Z 轴角（弧度）
   * @param[out] Y 输出的绕固定 Y 轴角（弧度）
   * @param[out] X 输出的绕固定 X 轴角（弧度）
   * @details 在 Y = +/- 90 deg 的退化情况下会记录 DEBUG 日志
   */
  void ToFixedZyx(T* Z, T* Y, T* X) const {
    RotationMatrix3<T> m = ToRotationMatrix();
    T sqr = std::sqrt(m(1, 2) * m(1, 2) + m(2, 2) * m(2, 2));
    *Y = static_cast<T>(std::atan2(m(0, 2), sqr));

    constexpr T kEps4 = static_cast<T>(1e-4);
    T pi_half = static_cast<T>(M_PI) * static_cast<T>(0.5);

    if (std::abs(*Y - pi_half) <= kEps4) {
      std::fprintf(stderr, "[DEBUG] Fixed ZYX degenerate at +90 deg Y\n");
      *Z = 0;
      *X = static_cast<T>(std::atan2(m(1, 0), m(1, 1)));
    } else if (std::abs(*Y + pi_half) <= kEps4) {
      std::fprintf(stderr, "[DEBUG] Fixed ZYX degenerate at -90 deg Y\n");
      *Z = 0;
      *X = -static_cast<T>(std::atan2(m(1, 0), m(1, 1)));
    } else {
      *Z = static_cast<T>(std::atan2(-m(0, 1), m(0, 0)));
      *X = static_cast<T>(std::atan2(-m(1, 2), m(2, 2)));
    }
  }

  /**
   * @brief 将四元数转换为 6x6 齐次变换矩阵（仅旋转部分）
   * @param[out] transform 6x6 输出数组，左上与右下 3x3 块填充为旋转矩阵，其余为零
   */
  void ToTransform6(T transform[6][6]) const {
    RotationMatrix3<T> rot = ToRotationMatrix();

    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        transform[i][j] = 0;
      }
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        transform[i][j] = static_cast<T>(rot(i, j));
        transform[i + 3][j + 3] = static_cast<T>(rot(i, j));
      }
    }
  }

  /**
   * @brief 判断两个四元数是否相等
   * @param[in] other 另一个四元数
   * @return 相等返回 true，否则返回 false
   */
  bool operator==(const Quaternion& other) const {
    return q0 == other.q0 && q1 == other.q1 &&
           q2 == other.q2 && q3 == other.q3;
  }

  /**
   * @brief 判断两个四元数是否不相等
   * @param[in] other 另一个四元数
   * @return 不相等返回 true，否则返回 false
   */
  bool operator!=(const Quaternion& other) const {
    return !(*this == other);
  }

  /**
   * @brief 取负运算符
   * @return 各分量取反后的四元数
   */
  Quaternion operator-() const {
    return Quaternion(-q0, -q1, -q2, -q3);
  }

  /**
   * @brief 四元数加法
   * @param[in] other 另一个四元数
   * @return 相加结果
   */
  Quaternion operator+(const Quaternion& other) const {
    return Quaternion(q0 + other.q0, q1 + other.q1,
                      q2 + other.q2, q3 + other.q3);
  }

  /**
   * @brief 四元数减法
   * @param[in] other 另一个四元数
   * @return 相减结果
   */
  Quaternion operator-(const Quaternion& other) const {
    return Quaternion(q0 - other.q0, q1 - other.q1,
                      q2 - other.q2, q3 - other.q3);
  }

  /**
   * @brief 四元数 Hamilton 乘积
   * @param[in] other 另一个四元数
   * @return 乘积结果，结果会强制标量部分为正
   */
  Quaternion operator*(const Quaternion& other) const {
    T dot = q1 * other.q1 + q2 * other.q2 + q3 * other.q3;
    T r0 = q0 * other.q0 - dot;
    T r1 = q0 * other.q1 + q1 * other.q0 + q2 * other.q3 - q3 * other.q2;
    T r2 = q0 * other.q2 + q2 * other.q0 + q3 * other.q1 - q1 * other.q3;
    T r3 = q0 * other.q3 + q3 * other.q0 + q1 * other.q2 - q2 * other.q1;
    Quaternion result(r0, r1, r2, r3);
    result.EnforcePositiveScalar();
    return result;
  }

  /**
   * @brief 四元数与标量相乘
   * @param[in] scalar 标量
   * @return 缩放后的四元数
   */
  Quaternion operator*(T scalar) const {
    return Quaternion(q0 * scalar, q1 * scalar,
                      q2 * scalar, q3 * scalar);
  }

  /**
   * @brief 四元数除以标量
   * @param[in] scalar 标量
   * @return 除法结果；若 scalar 为零则记录 ERROR 日志并回退返回原四元数
   */
  Quaternion operator/(T scalar) const {
    if (scalar == 0) {
      std::fprintf(stderr, "[ERROR] Quaternion division by zero, returning original quaternion\n");
      return *this;
    }
    T inv = static_cast<T>(1) / scalar;
    return Quaternion(q0 * inv, q1 * inv, q2 * inv, q3 * inv);
  }

  /**
   * @brief 四元数自乘标量
   * @param[in] scalar 标量
   * @return 自身的引用
   */
  Quaternion& operator*=(T scalar) {
    q0 *= scalar; q1 *= scalar; q2 *= scalar; q3 *= scalar;
    return *this;
  }

  /**
   * @brief 四元数自除标量
   * @param[in] scalar 标量
   * @return 自身的引用；若 scalar 为零则记录 ERROR 日志并保持原值不变
   */
  Quaternion& operator/=(T scalar) {
    if (scalar == 0) {
      std::fprintf(stderr, "[ERROR] Quaternion division by zero, value unchanged\n");
      return *this;
    }
    T inv = static_cast<T>(1) / scalar;
    q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
    return *this;
  }

  /**
   * @brief 四元数自加
   * @param[in] other 另一个四元数
   * @return 自身的引用
   */
  Quaternion& operator+=(const Quaternion& other) {
    q0 += other.q0; q1 += other.q1; q2 += other.q2; q3 += other.q3;
    return *this;
  }

  /**
   * @brief 四元数自减
   * @param[in] other 另一个四元数
   * @return 自身的引用
   */
  Quaternion& operator-=(const Quaternion& other) {
    q0 -= other.q0; q1 -= other.q1; q2 -= other.q2; q3 -= other.q3;
    return *this;
  }

  /**
   * @brief 四元数自乘（Hamilton 乘积）
   * @param[in] other 另一个四元数
   * @return 自身的引用
   */
  Quaternion& operator*=(const Quaternion& other) {
    *this = *this * other;
    return *this;
  }
};

/**
 * @brief 标量左乘四元数
 * @param[in] scalar 标量
 * @param[in] q 四元数
 * @return 缩放后的四元数
 */
template <typename T>
Quaternion<T> operator*(T scalar, const Quaternion<T>& q) {
  return q * scalar;
}

}  // namespace math

using math::Cartesian3;
using math::Quaternion;
using math::RotationMatrix3;

#endif  // QUATERNION_HPP_
