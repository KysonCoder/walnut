// func/pathplanning_humanoid/include/whole_body_path_assembler.hpp
#ifndef FUNC_PATHPLANNING_HUMANOID_INCLUDE_WHOLE_BODY_PATH_ASSEMBLER_HPP_
#define FUNC_PATHPLANNING_HUMANOID_INCLUDE_WHOLE_BODY_PATH_ASSEMBLER_HPP_

#include <string>
#include <vector>

#include "dyn_humanoid_types.hpp"
#include "humanoid_path.hpp"
#include "pphumanoid_types.hpp"
#include "trajectory_types.hpp"   // SOC_UNIT_TYPE

namespace action {
class KinematicsResolver;
}

namespace pphumanoid {

// 全身 q(s) 组装器：多臂笛卡尔路径按弧长归一到共享 s∈[0,1]，
// 逐点 IK（ref 链式播种保证分支连续）→ 19-DOF 采样表 + 数值微分。
// 腰部走关节空间线性插值（不走 IK——kinplanning 后端是臂式解析 IK，
// 腰部 4 关节不在其适用范围）；头部/未使能关节保持 q_current。
class WholeBodyPathAssembler {
 public:
  struct ArmInput {
    const HumanoidPath* path;        // 该臂笛卡尔路径
    dyn_humanoid::Chain chain;       // kLeftArm / kRightArm
    SOC_UNIT_TYPE soc;               // LEFT_ARM / RIGHT_ARM（CTarget::Attach 用）
  };

  // waist_joints_end: 腰部 4 关节终点（关节空间线性插值）；nullptr = 腰静止。
  // strict_ik: true 时任一点 IK 失败返回 PPH_ERR_IK_FAILED；
  //            false 时复用上一点关节角继续（调用方记 warning）。
  int Assemble(const std::vector<ArmInput>& arms,
               const dyn_humanoid::VecW& q_current,
               const double* waist_joints_end,
               const action::KinematicsResolver& resolver,
               int num_samples, bool strict_ik,
               WholeBodyPathSamples& out, std::string& err) const;
};

}  // namespace pphumanoid

#endif  // FUNC_PATHPLANNING_HUMANOID_INCLUDE_WHOLE_BODY_PATH_ASSEMBLER_HPP_
