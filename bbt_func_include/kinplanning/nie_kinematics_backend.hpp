#ifndef NIE_KINEMATICS_BACKEND_HPP
#define NIE_KINEMATICS_BACKEND_HPP

#include <memory>

#include "kinematics_backend.hpp"
#include "nie_group.hpp"
#include "nie_target.hpp"

namespace action {

/**
 * @brief NIE dual-arm kinematics backend.
 *
 * Wraps func/kinematic's CNieTarget and reads the waist theta from an attached
 * CNieGroup. If no CNieGroup is attached before Init(), a private fallback
 * group is allocated for standalone/testing use.
 */
class NieKinematicsBackend : public IKinematicsBackend {
 public:
  NieKinematicsBackend();
  ~NieKinematicsBackend() override = default;

  bool Init(const kinematic::KinematicConfigAdapter& adapter) override;
  bool ResolvePose(CTarget& target) override;
  bool ResolveJoints(CTarget& target, const CTarget& ref) override;
  const char* BackendName() const override { return "NieKinematicsBackend"; }

  // Allows PathPlanningPipeline / TrajectoryPlanner to attach an externally
  // managed target group (must point to a CNieGroup in this flavor).
  // Calling this after Init() replaces any fallback group.
  void AttachGroup(CTargetGroup* group);
  // 旧名兼容（下一轮删除）。
  void AttachNieGroup(CNieGroup* nie_group) { AttachGroup(nie_group); }

 private:
  // NIE 专属访问处 static_cast 回 CNieGroup*（本 backend 仅 dualarm 编译）。
  CNieGroup* NieGroup() { return static_cast<CNieGroup*>(group_); }
  const CNieGroup* NieGroup() const { return static_cast<const CNieGroup*>(group_); }

  CTargetGroup* group_ = nullptr;
  std::unique_ptr<CNieGroup> owned_group_;
  std::unique_ptr<TargetGroupSharedData> owned_data_;
  interpolation::CNieTarget solver_;
};

}  // namespace action

#endif
