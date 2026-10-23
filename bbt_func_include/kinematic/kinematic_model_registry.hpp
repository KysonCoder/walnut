#ifndef FUNC_KINEMATIC_INCLUDE_KINEMATIC_MODEL_REGISTRY_HPP_
#define FUNC_KINEMATIC_INCLUDE_KINEMATIC_MODEL_REGISTRY_HPP_

#include <cstddef>

namespace kinematic {

enum class KinematicModelClass {
  kArmElbow,
  kArmIrb,
  kFinger,
  kThumb,
  kNone,
  kUnknown,
  kJointPassthrough,  // 2026-09-16 FR2: 纯关节透传（无运动学模型 unit）
  kArmNie,   // 2026-10-08: "CNieTarget" — NIE 双臂机器人单臂求解器（CNieGroup 协调）
  kNieWaist  // 2026-10-08: "CNieWaist" — NIE 1-DOF 腰（CNieGroup 驱动）
};

KinematicModelClass ClassifyKinematicModel(const char* name);

}  // namespace kinematic

#endif  // FUNC_KINEMATIC_INCLUDE_KINEMATIC_MODEL_REGISTRY_HPP_
