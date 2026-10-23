#ifndef HUMANOID_KINEMATICS_BACKEND_HPP
#define HUMANOID_KINEMATICS_BACKEND_HPP

#include "kinematics_backend.hpp"
#include "legacy_robot_types.hpp"

namespace action {

/**
 * @brief Humanoid / generic 6-DOF kinematics backend.
 *
 * Delegates FK/IK to func/kinematic's CArmTarget. The MAN_ROBOT parameters are
 * built once during Init() from the confbase kinematic parameters so that
 * callers do not need to supply them per target.
 */
class HumanoidKinematicsBackend : public IKinematicsBackend {
 public:
  HumanoidKinematicsBackend();
  ~HumanoidKinematicsBackend() override = default;

  bool Init(const kinematic::KinematicConfigAdapter& adapter) override;
  bool ResolvePose(CTarget& target) override;
  bool ResolveJoints(CTarget& target, const CTarget& ref) override;
  const char* BackendName() const override { return "HumanoidKinematicsBackend"; }

  bool GetKinematicContext(MAN_ROBOT* robot_param,
                           FRAME* tool_frame) const override;

 private:
  FRAME tool_frame_{};
  MAN_ROBOT robot_param_{};
  bool robot_param_valid_ = false;
};

}  // namespace action

#endif
