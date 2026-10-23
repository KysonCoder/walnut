#ifndef BASE_CONFBASE_INCLUDE_CONFBASE_DYNAMICS_CONFIG_ADAPTER_HPP_
#define BASE_CONFBASE_INCLUDE_CONFBASE_DYNAMICS_CONFIG_ADAPTER_HPP_

#include <cstdint>
#include <cstddef>

// RobotConfigStore 是 robot.hpp 中的 using 别名（当前 ROBOT_TYPE 的 typed
// store），无法前向声明为 class——直接包含定义头。
// （2026-08-24 修复：task 侧临时修补合入主干。）
#include "robot.hpp"

namespace robot_config {
class DualArmConfigStore;
class HumanoidConfigStore;
struct UnitConfig;
struct JointDriveParams;
struct RobotTopology;
}  // namespace robot_config

namespace robot_config {

class DynamicsConfigAdapter {
 public:
  DynamicsConfigAdapter();

  // Attach 入口。RobotConfigStore 是 robot.hpp 中当前 ROBOT_TYPE 的 typed
  // store 别名，本重载即该 typed store 的入口。
  bool Attach(RobotConfigStore& store);

  // 另一 typed store 的入口。RobotConfigStore 是当前 ROBOT_TYPE 的别名，
  // 同型重载须屏蔽，否则签名重复（2026-08-24 修复）。
#if ROBOT_TYPE == HUMANOID
  bool Attach(DualArmConfigStore& store);
#else
  bool Attach(HumanoidConfigStore& store);
#endif

  void Detach();
  bool IsAttached() const;

  bool GetUrdfString(const char** out_urdf, size_t* out_size) const;
  bool GetUnitConfig(int32_t unit_type, const UnitConfig** out_unit) const;
  bool GetJointDriveParams(int32_t unit_type, int32_t joint_index,
                           const JointDriveParams** out_params) const;
  bool GetRobotTopology(const RobotTopology** out_topology) const;
  bool GetDynGroupNumber(int32_t unit_type, int32_t dyngrp_model,
                         int32_t* out_dyngrp_no) const;
  bool GetPowerRegainLimit(int32_t unit_type, int32_t dyngrp_model,
                           double* out_limit) const;

 private:
  enum class StoreType { kNone, kDualArm, kHumanoid };

  StoreType store_type_ = StoreType::kNone;
  const void* store_ = nullptr;
};

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_CONFBASE_DYNAMICS_CONFIG_ADAPTER_HPP_
