#ifndef UNIFIED_DYNAMICS_MODEL_HPP
#define UNIFIED_DYNAMICS_MODEL_HPP
#include "confbase/robot.hpp"
#if ROBOT_TYPE == DUAL_ARM

#include "path_action_types.hpp"
#include "composite_robot_model.hpp"
#include "dynamics_config_adapter.hpp"
#include "path_planning_config.hpp"
#include "dual_arm_dyn.hpp"
#include <memory>
#include <string>
#include <vector>

namespace action {

struct UnifiedDynamicsResult {
  std::vector<double> s;
  std::vector<double> ds_dt;
  std::vector<double> d2s_dt2;
  std::vector<double> timestamp;
  bool feasible{false};
};

class UnifiedDynamicsModel {
 public:
  UnifiedDynamicsModel();
  ~UnifiedDynamicsModel();

  bool Init(const robot_config::DynamicsConfigAdapter& adapter,
            const CompositeRobotModel& model,
            const std::string& base_link,
            const std::string& body_link);

  bool Optimize(const std::vector<PathActionSequence*>& soc_paths,
                const PathPlanningConfig& config,
                UnifiedDynamicsResult& out_timing);

  bool IsInitialized() const { return dual_arm_dyn_ != nullptr; }

 private:
  std::unique_ptr<interpolation::DualArmDyn> dual_arm_dyn_;
  CompositeRobotModel::SocDesc left_desc_;
  CompositeRobotModel::SocDesc right_desc_;
};

}  // namespace action

#endif  // UNIFIED_DYNAMICS_MODEL_HPP
#endif  // ROBOT_TYPE == DUAL_ARM
