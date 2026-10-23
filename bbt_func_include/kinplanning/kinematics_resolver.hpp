#ifndef KINEMATICS_RESOLVER_HPP
#define KINEMATICS_RESOLVER_HPP

#include "kinematics_backend.hpp"
#include "target.hpp"
#include "math/pose.hpp"
// confbase 2026-08-24: the generic RobotConfigStore class conflict is fixed
// root-side (adapter header now includes robot.hpp); the flavor alias is safe
// to use everywhere.
#include "confbase/robot.hpp"
#include <memory>

namespace kinematic {
class KinematicConfigAdapter;
}  // namespace kinematic

class CNieGroup;
class CTargetGroup;

namespace action {

/**
 * @brief Resolves kinematics (FK/IK) for CTarget using a pluggable backend.
 *
 * The concrete backend (Humanoid or NIE) is selected automatically from the
 * confbase configuration supplied to Init().
 */
class KinematicsResolver {
 public:
  struct Config {
    int max_ik_iterations{100};
    double ik_position_tol{1e-4};
    double ik_orient_tol{1e-3};
    double ik_step_size{0.5};
  };

  KinematicsResolver();
  explicit KinematicsResolver(std::unique_ptr<IKinematicsBackend> backend);

  bool Init(const kinematic::KinematicConfigAdapter& adapter,
            const Config& cfg);
  bool Init(const kinematic::KinematicConfigAdapter& adapter) {
    return Init(adapter, Config{});
  }

  // Convenience initializer that attaches to the global flavor config store
  // (DualArmConfigStore or HumanoidConfigStore via the robot.hpp alias).
  bool Init(robot_config::RobotConfigStore& store, const Config& cfg);
  bool Init(robot_config::RobotConfigStore& store) {
    return Init(store, Config{});
  }

  // Attach an externally-managed target group (required for NIE before
  // Resolve*). Base-pointer interface: any CTargetGroup-derived group.
  void AttachGroup(CTargetGroup* group);
#if ROBOT_TYPE == DUAL_ARM
  // 旧名兼容（下一轮删除；NIE 仅 dualarm，别名只在 dualarm 编译）。
  void AttachNieGroup(CNieGroup* nie_group);
#endif

  bool ResolveJoints(CTarget& target, const CTarget& ref) const;
  bool ResolveJoints(CTarget& target) const;
  bool ResolvePose(CTarget& target) const;

  // Check if target has valid joint angles (based on is_joints_valid flag;
  // all-zero joints are a legal configuration).
  static bool HasValidJoints(const CTarget& target);

  // Check if target has valid Cartesian pose.
  static bool HasValidPose(const CTarget& target);

  bool IsInitialized() const { return backend_ != nullptr; }
  const char* BackendName() const {
    return backend_ ? backend_->BackendName() : "null";
  }

  // Forward kinematic context from backend (false if backend does not provide).
  bool GetKinematicContext(MAN_ROBOT* robot_param, FRAME* tool_frame) const;

 private:
  std::unique_ptr<IKinematicsBackend> backend_;
  Config config_;
};

}  // namespace action

#endif
