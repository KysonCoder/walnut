///@brief Robot target class
///
/// Holds robot pose (position, orientation, configuration) and joint angles.
/// Provides forward/inverse kinematics, Jacobian computation, and
/// configuration checking.
///
/// Data is stored externally via TargetData pointer (inherited from CBaseTarget).
///
///@author Yonglin
///@version 0.2
///@date 2026-05-08

#ifndef _ROB_ARM_TARGET_H_
#define _ROB_ARM_TARGET_H_

#include "confbase/humanoid_conf.hpp"
#include "legacy_robot_types.hpp"
#include "quaternion.hpp"
#include "pose.hpp"
#include "basetarget.hpp"
#include "kinematic_error_info.hpp"  // ROBOT_TARGET_* 统一码表现居 basetarget.hpp（2026-10-10 下沉）



#define MATH_LIB_CARTESIAN_NULL_VECTOR -1
#define ROBOT_TARGET_INIT {{0,},}

///@brief Elbow-type robot checkpoint positions.
struct ElbowTarget {
  math::Cartesian3<double> cp_arm;
  math::Cartesian3<double> cp_wcp;
};

///@brief IRB-type robot checkpoint positions.
struct IrbTarget {
  math::Cartesian3<double> cp_arm;
  math::Cartesian3<double> cp_wcp;
};

namespace interpolation {

///@brief Robot kinematic model type.
enum RobotType {
  kIrbRobot,    ///< IRB-type parallel link robot (CKinPLinkRbt).
  kElbowRobot,  ///< Elbow-type robot (CKinGeneRbt).
  kNieRobot     ///< NIE dual-arm single-arm solver (CNieTarget).
};

///@brief Robot target holding pose, joints, and kinematic state.
/// All data is stored externally via TargetData pointer in CBaseTarget.
class CArmTarget : public CBaseTarget {
 public:
  CArmTarget();
  explicit CArmTarget(RobotType rt);
  explicit CArmTarget(TargetData* data);
  ~CArmTarget();

  ///@name Arm-specific field accessors
  ///@{

  void SetArmAngle(double arm_angle_in);
  double GetArmAngle() const;

  void SetRobotType(RobotType rt);
  RobotType GetRobotType() const;

  /// 注入腰槽 TargetData（1-DOF robot_body，joints[0]=腰角），供 kNieRobot
  /// 委派路径读取。视图成员，非 POD，不影响 shm 布局。CNieGroup::Attach 自动注入。
  void SetWaistTargetData(const TargetData* waist_data);

  void SetArmName(const char* name);
  void GetArmName(char* name) const;

  void SetCheckpoints(const math::Cartesian3<double>& cp_arm,
                      const math::Cartesian3<double>& cp_wcp);
  void GetCheckpoints(const MAN_ROBOT& robot_param,
                      math::Cartesian3<double>& cp_arm,
                      math::Cartesian3<double>& cp_wcp) const;
  void GetCheckpoints(math::Cartesian3<double>& cp_arm,
                      math::Cartesian3<double>& cp_wcp) const;

  ///@name Direct data accessors (for internal/kinematic module use)
  ///@{
  TARGET_CONFIG& Config() { return _data->config; }
  const TARGET_CONFIG& Config() const { return _data->config; }

  POSE& Pose() { return _data->pose; }
  const POSE& Pose() const { return _data->pose; }
  ///@}

  ///@}

  ///@name Pose accessors (unified via base class TargetData)
  ///@{

  void SetPose(const math::Cartesian3<double>& position,
               const math::Quaternion<double>& orientation,
               const TARGET_CONFIG& config_in);

  void GetPose(math::Cartesian3<double>& position,
               math::Quaternion<double>& orientation,
               TARGET_CONFIG& config_in) const;

  ///@}

  ///@name Joint accessors
  ///@{

  void SetJoints(const SOC_JOINTS& joints_in);

  void SetJoint(int joint_no, double joint) {
     Joint(joint_no) = joint;
  }

  void SetConfig(const TARGET_CONFIG& config_in);

  void GetJoints(SOC_JOINTS& joints_in) const;

  void GetJoint(int joint_no, double& joint) const {
     joint = Joint(joint_no);
  }

  ///@}

  ///@name Forward kinematics
  ///@{

  int CalcPose(const MAN_ROBOT& robot_param,
               const FRAME& tool_frame,
               bool robot_fixed_object);

  int CalcPoseNom(const MAN_ROBOT& robot_param,
                  const FRAME& tool_frame,
                  bool robot_fixed_object);

  ///@}

  ///@name Inverse kinematics
  ///@{

  /// @param err 详情通道（可选，nullptr 零开销）。2026-10-10 报告 §3.2。
  int CalcJointCnf(const MAN_ROBOT& robot_param,
                   const CArmTarget& p_ref,
                   const FRAME& tool_frame,
                   bool robot_fixed_object,
                   KinematicErrorInfo* err = nullptr);

  int SearchJointCnf(const MAN_ROBOT& robot_param,
                     CArmTarget& p_ref,
                     const FRAME& tool_frame,
                     bool robot_fixed_object);

  /// @param err 详情通道（可选，nullptr 零开销）。
  int CalcJointNoCnf(const MAN_ROBOT& robot_param,
                     const CArmTarget& p_ref,
                     const FRAME& tool_frame,
                     bool robot_fixed_object,
                     KinematicErrorInfo* err = nullptr);

  int CalcJointNoCnfNom(const MAN_ROBOT& robot_param,
                        const CArmTarget& p_ref,
                        const FRAME& tool_frame,
                        bool robot_fixed_object);

  int CalcJointOldCnfNom(const MAN_ROBOT& robot_param,
                         const CArmTarget& p_ref,
                         const FRAME& tool_frame,
                         bool robot_fixed_object);

  int CalcJointCnfNom(CArmTarget& p,
                      const MAN_ROBOT& robot_param,
                      const CArmTarget& p_ref,
                      const FRAME& tool_frame,
                      bool robot_fixed_object);

  ///@}

  ///@name Partial orientation IK
  ///@{

  int CalcOri123NoCnf(CArmTarget& p,
                      const MAN_ROBOT& robot_param,
                      const CArmTarget& p_ref,
                      const FRAME& tool_frame,
                      bool robot_fixed_object);

  int CalcOri123NoCnfNom(CArmTarget& p,
                         const MAN_ROBOT& robot_param,
                         const CArmTarget& p_ref,
                         const FRAME& tool_frame,
                         bool robot_fixed_object);

  int CalcOri134NoCnfNom(CArmTarget& p,
                         const MAN_ROBOT& robot_param,
                         const CArmTarget& p_ref,
                         const FRAME& tool_frame,
                         bool robot_fixed_object);

  int CalcOri234NoCnf(CArmTarget& p,
                      const MAN_ROBOT& robot_param,
                      const CArmTarget& p_ref,
                      const FRAME& tool_frame,
                      bool robot_fixed_object);

  ///@}

  ///@name Configuration checking
  ///@{

  int CheckMasterCnf(const CArmTarget& tg1p,
                     const CArmTarget& tg2p,
                     const MAN_ROBOT& robot_param,
                     bool check_wrist_only,
                     double rotation_limit);

  ///@}

  ///@name Singularity and path pose
  ///@{

  /// 2026-10-10：bool→int（1=OK/负=错误/-203=该模型不支持）。
  int CalcJointSing(CArmTarget& p,
                     const MAN_ROBOT& robot_param,
                     const CArmTarget& p_ref,
                     const FRAME& tool_frame,
                     bool robot_fixed_object);

  bool CalcPathPose(const math::Cartesian3<double>& path_x,
                    math::Pose<double>& path_pose);

  ///@}

  ///@name Jacobian
  ///@{

  /// 2026-10-10：bool→int（统一码）。
  int CalcVelocityJacobian(double jacobian[6][6],
                           const MAN_ROBOT& robot_param);

  /// 2026-10-10：bool→int（统一码）。
  int CalcVelocityJacobianNom(double jacobian[6][6],
                              const MAN_ROBOT& robot_param);

  bool CalcMc6Jacobian(const MAN_ROBOT& mrp,
                       const double q_vector[9],
                       double wrist_length,
                       const double joint_offset[3],
                       double svd_tol,
                       double A39_jacobian[3][9]);

  ///@}

  ///@name Singularity detection
  ///@{

  bool HasWristSingularity(const MAN_ROBOT& robot_param);

  ///@}

 private:
  MAN_ROBOT BuildManRobotFromKinematicParams_() const;
  MAN_ROBOT BuildManRobotFromUrdf_(const char* urdf,
                                     const char* root_link,
                                     const char* tip_link) const;
  /// kNieRobot 委派路径的腰角：注入槽 joints[0]；未注入时 0.0 + Log_W 兜底。
  double NieWaistTheta_() const;

  const TargetData* waist_data_ = nullptr;  // 视图成员（非 shm 数据）
  MAN_ROBOT ResolveRobotParam_(const MAN_ROBOT& robot_param) const;
};

}  // namespace interpolation

#endif
