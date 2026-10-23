#ifndef NIE_WAIST_HPP_
#define NIE_WAIST_HPP_

#include "basetarget.hpp"
#include "armtarget.hpp"  // ROBOT_TARGET_* 错误码
#include "math/se3_twist_ops.hpp"

/**
 * @brief NIE 机器人 1-DOF 腰部目标。
 *
 * joints[0] 存储 waist theta（rad）。
 * unit_type 应为 kSocBody（dualarm 紧凑槽位 2 / humanoid legacy ROBOT_BODY）。
 */
class CNieWaist : public CBaseTarget {
 public:
  CNieWaist();
  explicit CNieWaist(TargetData* data);

  /// @return 1 成功；ROBOT_TARGET_JOINT_OUT_OF_RANGE 超限（值仍被钳制写入）；
  ///         ROBOT_TARGET_NOT_ATTACHED 未 Attach。
  int SetWaistTheta(double theta);
  double GetWaistTheta() const;
  se3_twist::Mat4 CalcWaistPose() const;

  /// 配置限位（confbase 驱动时由 group 注入；缺省 ±150°）。
  void SetLimits(double min_theta, double max_theta);

  static double MinTheta();  ///< 缺省下限（-150°）
  static double MaxTheta();  ///< 缺省上限（+150°）

 private:
  double min_theta_ = MinTheta();
  double max_theta_ = MaxTheta();
};

#endif  // NIE_WAIST_HPP_
