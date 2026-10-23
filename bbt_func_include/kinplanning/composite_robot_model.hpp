#ifndef COMPOSITE_ROBOT_MODEL_HPP
#define COMPOSITE_ROBOT_MODEL_HPP

#include "targetgroup.hpp"
#include "target.hpp"
#include <string>
#include <vector>

namespace action {

class CompositeRobotModel {
 public:
  struct SocDesc {
    int soc_index{0};
    int dof{6};
    std::string root_link;
    std::string tip_link;
    std::string kinematic_model_name;
  };

  CompositeRobotModel();
  ~CompositeRobotModel();

  bool Init(const CTargetGroup& prototype_group);

  int GetEnabledSocCount() const;
  const std::vector<SocDesc>& GetEnabledSocs() const;

  bool ExtractSocTarget(const CTargetGroup& group, int soc_index,
                        TargetData& out) const;

 private:
  std::vector<SocDesc> soc_descs_;
};

}  // namespace action

#endif  // COMPOSITE_ROBOT_MODEL_HPP
