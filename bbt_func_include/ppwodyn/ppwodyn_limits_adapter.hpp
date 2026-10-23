#ifndef FUNC_PPWODYN_INCLUDE_PPWODYN_LIMITS_ADAPTER_HPP_
#define FUNC_PPWODYN_INCLUDE_PPWODYN_LIMITS_ADAPTER_HPP_

#include "ppwodyn_config.hpp"
#include "confbase/dynamics_config_adapter.hpp"

namespace ppwodyn {

// 从 confbase/drivechain 读取运动学限幅（关节速度/加速度 + 末端线速度）。
// 拷贝自 pathplanning::PathPlanningLimitsAdapter，仅保留速度/加速度部分
// （FillTorqueLimits13 依赖动力学 DynamicPlanner::Config，已删除）。
class PpwodynLimitsAdapter {
 public:
  PpwodynLimitsAdapter();
  ~PpwodynLimitsAdapter();

  bool Attach(const robot_config::DynamicsConfigAdapter& adapter);
  bool IsAttached() const { return attached_; }

  // 只覆盖当前为 0 或等于库默认值的字段；用户正值保留。
  bool FillVelocityLimits(KinematicLimits* out) const;

 private:
  bool attached_ = false;
};

}  // namespace ppwodyn

#endif  // FUNC_PPWODYN_INCLUDE_PPWODYN_LIMITS_ADAPTER_HPP_
