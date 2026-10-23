#ifndef POSE_HPP_
#define POSE_HPP_

#include "quaternion.hpp"
#include <cstdio>
#include <cmath>

namespace math {

/**
 * @enum PoseDifference
 * @brief 用于描述两个 Pose 之间差异类型的枚举
 */
enum class PoseDifference {
  kEqual,           /**< @brief 位姿完全相同 */
  kPositionDiff,    /**< @brief 仅位置不同 */
  kOrientationDiff, /**< @brief 仅姿态不同 */
  kBothDiff         /**< @brief 位置与姿态均不同 */
};

/**
 * @class Pose
 * @brief 刚体位姿模板类，包含三维位置与四元数姿态
 * @tparam T 元素类型，默认为 double
 */
template <typename T = double>
class Pose {
 public:
  Cartesian3<T> pos;     /**< @brief 三维位置 */
  Quaternion<T> orient;  /**< @brief 四元数姿态 */

  /** @brief 默认构造函数，构造为单位位姿（零位置，单位姿态） */
  Pose() = default;

  /**
   * @brief 带参数构造函数
   * @param[in] position 三维位置
   * @param[in] orientation 四元数姿态
   */
  Pose(const Cartesian3<T>& position, const Quaternion<T>& orientation)
      : pos(position), orient(orientation) {}

  /**
   * @brief 构造单位位姿
   * @return 单位位姿（零位置，单位姿态）
   */
  static Pose Unity() {
    return Pose(Cartesian3<T>(), Quaternion<T>::Unity());
  }

  /**
   * @brief 构造无效位姿
   * @return 零位置、零四元数的位姿（用于表示错误或初始化占位）
   */
  static Pose Invalid() {
    return Pose(Cartesian3<T>(), Quaternion<T>::Zero());
  }

  /**
   * @brief 清除位姿，重置为单位位姿
   */
  void Clear() {
    pos = Cartesian3<T>();
    orient = Quaternion<T>::Unity();
  }

  /**
   * @brief 位姿乘法
   * @param[in] other 另一个位姿
   * @return 乘积结果：当前位姿作用于另一个位姿的复合变换
   * @details 若 other 定义在当前坐标系下，则结果表示其在同一参考系下的表示
   */
  Pose operator*(const Pose& other) const {
    Pose result;
    result.orient = orient * other.orient;
    result.pos = orient.Rotate(other.pos) + pos;
    return result;
  }

  /**
   * @brief 求位姿的逆
   * @return 逆位姿，满足 p * Inverse() = Identity
   */
  Pose Inverse() const {
    Pose result;
    result.orient = orient.Inverse();
    auto rot_pos = result.orient.Rotate(pos);
    result.pos = -rot_pos;
    return result;
  }

  /**
   * @brief 当前位姿乘以另一个位姿的逆
   * @param[in] other 另一个位姿
   * @return pr = *this * other.Inverse()
   */
  Pose MultInv(const Pose& other) const {
    return *this * other.Inverse();
  }

  /**
   * @brief 当前位姿的逆乘以另一个位姿
   * @param[in] other 另一个位姿
   * @return pr = Inverse() * other
   */
  Pose InvMult(const Pose& other) const {
    return Inverse() * other;
  }

  /**
   * @brief 由三个空间点和一个原点选择标志定义位姿
   * @param[in] point0 第一个点，定义局部 x' 轴起点
   * @param[in] point1 第二个点，定义局部 x' 轴方向
   * @param[in] point2 第三个点，用于定义平面和 y' 轴方向
   * @param[in] x_origin 原点选择：0=point0，1=point1，2=y' 轴过 point2
   * @return 定义成功返回 true；输入不合法时记录 ERROR/WARN 日志并返回 false
   */
  bool Define(const Cartesian3<T>& point0,
              const Cartesian3<T>& point1,
              const Cartesian3<T>& point2,
              int x_origin);

  /**
   * @brief 比较两个位姿
   * @param[in] other 另一个位姿
   * @return 差异类型枚举值
   */
  PoseDifference Compare(const Pose& other) const {
    bool orient_equal = orient == other.orient;
    bool pos_equal = pos == other.pos;
    if (orient_equal && pos_equal) return PoseDifference::kEqual;
    if (!orient_equal && !pos_equal) return PoseDifference::kBothDiff;
    if (!pos_equal) return PoseDifference::kPositionDiff;
    return PoseDifference::kOrientationDiff;
  }

  /**
   * @brief 计算速度雅可比矩阵（6x6）
   * @param[out] vel_jac 6x6 速度雅可比矩阵
   * @details 计算刚体上两点之间的速度映射关系，格式为
   *          [ R   p_x*R ]
   *          [ 0     R   ]
   *          参见 Craig《机器人学导论》第 154 页
   */
  void VelocityJacobian(T vel_jac[6][6]) const;

  /**
   * @brief 计算力雅可比矩阵（6x6）
   * @param[out] force_jac 6x6 力雅可比矩阵
   * @details 计算刚体上两点之间在静力平衡下的力/力矩映射关系，格式为
   *          [ R       0 ]
   *          [ p_x*R   R ]
   *          参见 Craig《机器人学导论》第 154 页
   */
  void ForceJacobian(T force_jac[6][6]) const;
};

/*------------------------- Private helper functions ---------------------*/

/**
 * @brief 由三个不共线点计算外接圆圆心
 * @tparam T 元素类型
 * @param[in] p1 第一个点
 * @param[in] p2 第二个点
 * @param[in] p3 第三个点
 * @return 外接圆圆心坐标；若三点共线或过于接近则记录 ERROR 日志并返回零向量
 */
template <typename T>
Cartesian3<T> CalcCircleCenter(const Cartesian3<T>& p1,
                               const Cartesian3<T>& p2,
                               const Cartesian3<T>& p3) {
  Cartesian3<T> p12 = p2 - p1;
  Cartesian3<T> p13 = p3 - p1;

  T a = p12.Length();
  T b = p13.Length();
  Cartesian3<T> p23 = p3 - p2;
  T c = p23.Length();

  T s = (a + b + c) / static_cast<T>(2);
  T area_sq = s * (s - a) * (s - b) * (s - c);
  if (area_sq <= static_cast<T>(0)) {
    std::fprintf(stderr, "[ERROR] Three points are collinear or too close, cannot calculate circle center\n");
    return Cartesian3<T>();
  }
  T r = static_cast<T>(0.25) * a * b * c / std::sqrt(area_sq);

  /* 保证 p12 为较短的边 */
  if (a > b) {
    Cartesian3<T> tmp = p13;
    p13 = p12;
    p12 = tmp;
    T tmp_len = a;
    a = b;
    b = tmp_len;
  }

  T ai = static_cast<T>(1) / a;
  T bi = static_cast<T>(1) / b;

  Cartesian3<T> pn12 = p12 * ai;
  Cartesian3<T> pn13 = p13 * bi;

  T c1 = pn12.Dot(pn12);
  Cartesian3<T> pr1 = pn13 * c1;

  T c2 = pn12.Dot(pn13);
  Cartesian3<T> pr2 = pn12 * c2;
  Cartesian3<T> k2 = pr1 - pr2;

  T kl = static_cast<T>(1) / k2.Length();
  k2 = k2 * kl;

  T d_sq = r * r - (a * a / static_cast<T>(4));
  if (d_sq < static_cast<T>(0)) {
    std::fprintf(stderr, "[WARN] Circle center calculation encountered negative discriminant, clamping to zero\n");
    d_sq = static_cast<T>(0);
  }
  T d = std::sqrt(d_sq);

  Cartesian3<T> p0(
      p1.x + static_cast<T>(0.5) * p12.x + d * k2.x,
      p1.y + static_cast<T>(0.5) * p12.y + d * k2.y,
      p1.z + static_cast<T>(0.5) * p12.z + d * k2.z);

  return p0;
}

/*------------------------- Pose member implementations ------------------*/

template <typename T>
bool Pose<T>::Define(const Cartesian3<T>& point0,
                     const Cartesian3<T>& point1,
                     const Cartesian3<T>& point2,
                     int x_origin) {
  constexpr T kEps4 = static_cast<T>(1e-4);
  constexpr T kMaxCosA = static_cast<T>(0.9998);

  Cartesian3<T> x_vector = point1 - point0;
  T x_vector_length = x_vector.Length();

  if (x_vector_length < kEps4) {
    std::fprintf(stderr, "[ERROR] X-axis start and end too close, cannot define pose\n");
    return false;
  }

  Cartesian3<T> tp = point2 - point0;
  T tp_length = tp.Length();

  Cartesian3<T> z_vector = x_vector.Cross(tp);
  T z_vector_length = z_vector.Length();

  if (tp_length < kEps4) {
    std::fprintf(stderr, "[ERROR] Third point too close to start or end point\n");
    return false;
  }

  T dot_val = x_vector.Dot(tp) / x_vector_length;
  if (std::abs(dot_val) > kMaxCosA * tp_length) {
    std::fprintf(stderr, "[ERROR] Three input points are collinear, collapsed frame\n");
    return false;
  }

  Cartesian3<T> y_vector = z_vector.Cross(x_vector);
  T y_vector_length = y_vector.Length();

  RotationMatrix3<T> m_inv;
  m_inv(0, 0) = x_vector.x / x_vector_length;
  m_inv(0, 1) = x_vector.y / x_vector_length;
  m_inv(0, 2) = x_vector.z / x_vector_length;

  m_inv(1, 0) = y_vector.x / y_vector_length;
  m_inv(1, 1) = y_vector.y / y_vector_length;
  m_inv(1, 2) = y_vector.z / y_vector_length;

  m_inv(2, 0) = z_vector.x / z_vector_length;
  m_inv(2, 1) = z_vector.y / z_vector_length;
  m_inv(2, 2) = z_vector.z / z_vector_length;

  Cartesian3<T> point0_to_base = -(m_inv * point0);
  Cartesian3<T> point2_rel_frame = m_inv * point2 + point0_to_base;

  if (point2_rel_frame.y < static_cast<T>(0)) {
    m_inv(1, 0) = -m_inv(1, 0);
    m_inv(1, 1) = -m_inv(1, 1);
    m_inv(1, 2) = -m_inv(1, 2);
    point0_to_base.y = -point0_to_base.y;

    m_inv(2, 0) = -m_inv(2, 0);
    m_inv(2, 1) = -m_inv(2, 1);
    m_inv(2, 2) = -m_inv(2, 2);
    point0_to_base.z = -point0_to_base.z;
  }

  RotationMatrix3<T> m = m_inv.Transpose();
  orient = Quaternion<T>::FromRotationMatrix(m);

  switch (x_origin) {
    case 0:
      pos = point0;
      break;
    case 1:
      pos = point1;
      break;
    case 2: {
      Cartesian3<T> point2_rel = point2_rel_frame;
      point2_rel.y = static_cast<T>(0);
      point2_rel.z = static_cast<T>(0);
      Cartesian3<T> temp = point0_to_base - point2_rel;
      pos = m * temp;
      pos = -pos;
      break;
    }
    default:
      std::fprintf(stderr, "[ERROR] Unknown origin switch: %d\n", x_origin);
      return false;
  }

  return true;
}

template <typename T>
void Pose<T>::VelocityJacobian(T vel_jac[6][6]) const {
  RotationMatrix3<T> rot = orient.ToRotationMatrix();

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      vel_jac[i][j] = static_cast<T>(0);
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      vel_jac[i][j] = rot(i, j);
      vel_jac[i + 3][j + 3] = rot(i, j);
    }
  }

  vel_jac[0][3] = -pos.z * rot(1, 0) + pos.y * rot(2, 0);
  vel_jac[0][4] = -pos.z * rot(1, 1) + pos.y * rot(2, 1);
  vel_jac[0][5] = -pos.z * rot(1, 2) + pos.y * rot(2, 2);

  vel_jac[1][3] =  pos.z * rot(0, 0) - pos.x * rot(2, 0);
  vel_jac[1][4] =  pos.z * rot(0, 1) - pos.x * rot(2, 1);
  vel_jac[1][5] =  pos.z * rot(0, 2) - pos.x * rot(2, 2);

  vel_jac[2][3] = -pos.y * rot(0, 0) + pos.x * rot(1, 0);
  vel_jac[2][4] = -pos.y * rot(0, 1) + pos.x * rot(1, 1);
  vel_jac[2][5] = -pos.y * rot(0, 2) + pos.x * rot(1, 2);
}

template <typename T>
void Pose<T>::ForceJacobian(T force_jac[6][6]) const {
  RotationMatrix3<T> rot = orient.ToRotationMatrix();

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      force_jac[i][j] = static_cast<T>(0);
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      force_jac[i][j] = rot(i, j);
      force_jac[i + 3][j + 3] = rot(i, j);
    }
  }

  force_jac[3][0] = -pos.z * rot(1, 0) + pos.y * rot(2, 0);
  force_jac[3][1] = -pos.z * rot(1, 1) + pos.y * rot(2, 1);
  force_jac[3][2] = -pos.z * rot(1, 2) + pos.y * rot(2, 2);

  force_jac[4][0] =  pos.z * rot(0, 0) - pos.x * rot(2, 0);
  force_jac[4][1] =  pos.z * rot(0, 1) - pos.x * rot(2, 1);
  force_jac[4][2] =  pos.z * rot(0, 2) - pos.x * rot(2, 2);

  force_jac[5][0] = -pos.y * rot(0, 0) + pos.x * rot(1, 0);
  force_jac[5][1] = -pos.y * rot(0, 1) + pos.x * rot(1, 1);
  force_jac[5][2] = -pos.y * rot(0, 2) + pos.x * rot(1, 2);
}

/*------------------------- Free functions -------------------------------*/

/**
 * @brief 当机器人在圆弧运动中间停止并向后运动时，计算新的圆弧点
 * @tparam T 元素类型
 * @param[in] org_startpoint 原始圆弧起点
 * @param[in] org_cirpoint 原始圆弧中间点
 * @param[in] org_topoint 原始圆弧终点
 * @param[in] new_startpoint 新的起始点（后退后的停止位置）
 * @param[out] new_cirpoint 输出的新圆弧点
 * @return 计算成功返回 true；出现退化情况时记录日志并返回 false
 */
template <typename T>
bool CalcNewCirclePoint(const Cartesian3<T>& org_startpoint,
                        const Cartesian3<T>& org_cirpoint,
                        const Cartesian3<T>& org_topoint,
                        const Cartesian3<T>& new_startpoint,
                        Cartesian3<T>* new_cirpoint) {
  constexpr T kEps6 = static_cast<T>(1e-6);
  constexpr T kPi = static_cast<T>(3.1415926);
  constexpr T kCircleMinLength = static_cast<T>(0.003);
  constexpr T kMaxLengthBeforeCircle = static_cast<T>(0.02);

  Cartesian3<T> ptemp1 = org_cirpoint - org_startpoint;
  Cartesian3<T> ptemp2 = new_startpoint - org_startpoint;

  T dist = ptemp1.Dot(ptemp2);

  if (dist < static_cast<T>(0)) {
    if (std::abs(dist) < kMaxLengthBeforeCircle) {
      std::fprintf(stderr, "[WARN] Stop point too close before circle start\n");
      return false;
    }
    std::fprintf(stderr, "[INFO] Stop point is before circle start\n");
    return false;
  }

  T new_start_dist = ptemp2.Length();
  if (new_start_dist > kCircleMinLength) {
    Cartesian3<T> psc = CalcCircleCenter(org_startpoint, org_cirpoint, org_topoint);
    Cartesian3<T> ptemp_rad = org_startpoint - psc;
    T n_radius = ptemp_rad.Length();

    if (n_radius < kEps6) {
      std::fprintf(stderr, "[ERROR] Circle radius too small, cannot calculate new circle point\n");
      return false;
    }

    Cartesian3<T> ps1 = new_startpoint - org_startpoint;
    Cartesian3<T> ps2(
        ps1.x * static_cast<T>(0.5) + org_startpoint.x,
        ps1.y * static_cast<T>(0.5) + org_startpoint.y,
        ps1.z * static_cast<T>(0.5) + org_startpoint.z);
    Cartesian3<T> ps3 = ps2 - psc;

    T m_factor = n_radius / ps3.Length();
    new_cirpoint->x = psc.x + ps3.x * m_factor;
    new_cirpoint->y = psc.y + ps3.y * m_factor;
    new_cirpoint->z = psc.z + ps3.z * m_factor;

    Cartesian3<T> vec_start = org_startpoint - psc;
    Cartesian3<T> vec_end = org_topoint - psc;
    T ratio = vec_start.Dot(vec_end) / (n_radius * n_radius);
    T full_arc = static_cast<T>(0);
    if (std::abs(ratio) <= static_cast<T>(1)) {
      full_arc = static_cast<T>(std::acos(ratio));
    }

    Cartesian3<T> vec_cir = org_cirpoint - psc;
    T ratio1 = vec_start.Dot(vec_cir) / (n_radius * n_radius);
    T part_arc1 = static_cast<T>(0);
    if (std::abs(ratio1) <= static_cast<T>(1)) {
      part_arc1 = static_cast<T>(std::acos(ratio1));
    }

    T ratio2 = vec_cir.Dot(vec_end) / (n_radius * n_radius);
    T part_arc2 = static_cast<T>(0);
    if (std::abs(ratio2) <= static_cast<T>(1)) {
      part_arc2 = static_cast<T>(std::acos(ratio2));
    }

    if (std::abs(full_arc - part_arc1 - part_arc2) > static_cast<T>(0.01)) {
      full_arc = static_cast<T>(2) * kPi - full_arc;
    }

    if (std::abs(full_arc) > kPi) {
      /* 重新计算新圆弧扇区角 */
      Cartesian3<T> vec_new = new_startpoint - psc;
      T ratio_new = vec_end.Dot(vec_new) / (n_radius * n_radius);
      T part_arc1_new = static_cast<T>(0);
      if (std::abs(ratio_new) <= static_cast<T>(1)) {
        part_arc1_new = static_cast<T>(std::acos(ratio_new));
      }

      T ratio_new2 = vec_start.Dot(vec_new) / (n_radius * n_radius);
      T part_arc2_new = static_cast<T>(0);
      if (std::abs(ratio_new2) <= static_cast<T>(1)) {
        part_arc2_new = static_cast<T>(std::acos(ratio_new2));
      }

      T new_full_arc = static_cast<T>(0);
      if (std::abs(full_arc - (static_cast<T>(2) * kPi - part_arc1_new) - part_arc2_new)
          < static_cast<T>(0.01)) {
        new_full_arc = full_arc - (static_cast<T>(2) * kPi - part_arc1_new);
      } else {
        new_full_arc = full_arc - part_arc1_new;
      }

      if (std::abs(new_full_arc) > kPi) {
        /* 大于 180 度时取对称解 */
        new_cirpoint->x = static_cast<T>(2) * psc.x - new_cirpoint->x;
        new_cirpoint->y = static_cast<T>(2) * psc.y - new_cirpoint->y;
        new_cirpoint->z = static_cast<T>(2) * psc.z - new_cirpoint->z;
      }
    }
  } else {
    std::fprintf(stderr, "[WARN] New start point too close to original start point\n");
    return false;
  }

  return true;
}

}  // namespace math

using math::Pose;
using math::PoseDifference;
using math::CalcCircleCenter;
using math::CalcNewCirclePoint;

#endif  // POSE_HPP_
