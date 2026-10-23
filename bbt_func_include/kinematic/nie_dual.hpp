/**
 * @file nie_dual_chain.hpp
 * @brief NieNieDual 13-DOF 双臂协调器
 *
 * 三部分组合:
 *   第1部分: 腰部 1-DOF → waist_target.hpp
 *   第2部分: 左臂 6-DOF → mdh_arm_kin.hpp (MDH FK + 几何法Jac)
 *   第3部分: 右臂 6-DOF → mdh_arm_kin.hpp (相同算法, 不同基座)
 *
 * FK:  world_pose = T_waist × T_arm_mdh
 * Jac: 6×13 = [腰列(6×1) | 左臂列(6×6) | 0 ]
 *      6×13 = [腰列(6×1) | 0           | 右臂列(6×6)]
 *
 * 不修改任何原有框架代码, 全部独立新增。
 *
 * @date 2026-06-16
 */

#ifndef NIE_DUAL_HPP_
#define NIE_DUAL_HPP_

#include "nie_arm_conf.hpp"
#include "mdh_arm_kin.hpp"
#include "mdh_arm_ik.hpp"
#include "waist_target.hpp"

#include "math/pose.hpp"
#include "math/se3_twist_ops.hpp"

class CNieDual {
public:
    CNieDual();

    // ── 正运动学 (FK) ───────────────────────────────────
    /**
     * @brief 13关节 → 双臂末端位姿 (世界坐标系)
     * @param joints[13]  [theta0, theta1..6, theta7..12] (rad)
     * @param left_pose   输出 左臂末端世界位姿
     * @param right_pose  输出 右臂末端世界位姿
     */
    void CalcPose(const double joints[NIE_TOTAL_JOINTS],
                  math::Pose<double>& left_pose,
                  math::Pose<double>& right_pose);

    // ── 速度雅可比 (Jacobian, 几何法) ────────────────────
    /**
     * @brief 12×13 雅可比矩阵
     * @param joints[13]  关节角
     * @param jac_left[6][13]   输出 左臂末端对13关节的雅可比
     * @param jac_right[6][13]  输出 右臂末端对13关节的雅可比
     *
     * 行0-2: 角速度 [ωx, ωy, ωz]
     * 行3-5: 线速度 [vx, vy, vz]
     * 列: 关节0(腰) + 关节1-6(左臂) + 关节7-12(右臂)
     */
    void CalcJacobian(const double joints[NIE_TOTAL_JOINTS],
                      double jac_left[6][NIE_TOTAL_JOINTS],
                      double jac_right[6][NIE_TOTAL_JOINTS]);

    // ── 逆运动学 (IK) ───────────────────────────────────
    /**
     * @brief 第一类IK: theta0给定, 求theta1~theta12
     * @param left_target    左臂目标位姿 (世界坐标系)
     * @param right_target   右臂目标位姿 (世界坐标系)
     * @param theta0         腰部关节角 (rad)
     * @param ref_joints[13] 参考关节角 (用于时序一致性, nullptr=默认零位)
     * @param joints[13]     输出 13关节角
     * @return 1 成功；ROBOT_TARGET_POSE_OUTSIDE_REACH 任一臂 IK 无解
     */
    int CalcJoint(const math::Pose<double>& left_target,
                  const math::Pose<double>& right_target,
                  double theta0,
                  const double ref_joints[NIE_TOTAL_JOINTS],
                  double joints[NIE_TOTAL_JOINTS]);

    /// 获取关节限位
    void GetJointLimits(double jmin[6], double jmax[6]) const;
    void SetJointLimits(const double jmin[6], const double jmax[6]);

    /// 腰部限位
    double WaistMin() const;
    double WaistMax() const;

    // ── static helpers (exposed for testing) ───────────
    static se3_twist::Mat4 MatMul(const se3_twist::Mat4& A,
                                   const se3_twist::Mat4& B);
    static se3_twist::Mat4 PoseToMat(const math::Pose<double>& pose);

    // ── 参数访问 ─────────────────────────────────────────
    const MdhArmParams& GetArmParams() const { return arm_params_; }

private:
    /// 左右臂共用相同的 MDH 几何参数
    MdhArmParams arm_params_;

    /// 关节限位 (单臂6-DOF)
    double joint_min_[6];
    double joint_max_[6];

    /// 4×4 矩阵乘法 (用于 FK 中 T_waist × T_arm)
    static se3_twist::Mat4 MatMul_(const se3_twist::Mat4& A,
                                    const se3_twist::Mat4& B);

    /// 从 4×4 矩阵提取 Pose
    static math::Pose<double> MatToPose_(const se3_twist::Mat4& T);

    /// Pose → 4×4 矩阵
    static se3_twist::Mat4 PoseToMat_(const math::Pose<double>& pose);
};

#endif  // NIE_DUAL_HPP_
