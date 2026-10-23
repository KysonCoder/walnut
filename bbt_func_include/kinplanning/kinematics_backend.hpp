#ifndef KINEMATICS_BACKEND_HPP
#define KINEMATICS_BACKEND_HPP

#include "target.hpp"

namespace kinematic {
class KinematicConfigAdapter;
}  // namespace kinematic

namespace action {

/**
 * @brief Pure virtual interface for pluggable kinematics backends.
 *
 * Implementations provide forward/inverse kinematics for CTarget without
 * exposing the underlying solver (Humanoid elbow-IRB vs NIE dual-arm).
 */
class IKinematicsBackend {
 public:
  virtual ~IKinematicsBackend() = default;

  virtual bool Init(const kinematic::KinematicConfigAdapter& adapter) = 0;

  // FK: joints -> pose. Returns true and updates target validity flags on success.
  virtual bool ResolvePose(CTarget& target) = 0;

  // IK: pose -> joints, using ref to keep configuration continuity.
  virtual bool ResolveJoints(CTarget& target, const CTarget& ref) = 0;

  virtual const char* BackendName() const = 0;

  // Optional: expose kinematic context (robot params + tool frame) so callers
  // can invoke CTarget::CalcJoint directly. Default: unavailable.
  virtual bool GetKinematicContext(MAN_ROBOT* robot_param,
                                   FRAME* tool_frame) const {
    (void)robot_param;
    (void)tool_frame;
    return false;
  }
};

}  // namespace action

#endif
