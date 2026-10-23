// func/dyn_humanoid/include/humanoid_dyn_config_adapter.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_CONFIG_ADAPTER_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_CONFIG_ADAPTER_HPP_

#include <cstddef>

#include "confbase/dynamics_config_adapter.hpp"

namespace robot_config {
class HumanoidConfigStore;
}

namespace dyn_humanoid {

// robot_config::DynamicsConfigAdapter 的 HUMANOID 语义轻封装。
// 阶段二将在此补充 drivechain 力矩限幅提取（thermal/stress 表 → VecW）。
class HumanoidDynConfigAdapter {
 public:
  bool Attach(robot_config::HumanoidConfigStore& store);
  void Detach();
  bool IsAttached() const;

  bool GetUrdfString(const char** out_urdf, size_t* out_size) const;

 private:
  robot_config::DynamicsConfigAdapter inner_;
};

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_HUMANOID_DYN_CONFIG_ADAPTER_HPP_
