///@brief Define the 6DOF elbow robot inverse and forward kinematic
///@author Yonglin
///@version 0.1
///@date 18-Apr-2026

#ifndef _ROB_KIN_GENE_RBT_H_
#define _ROB_KIN_GENE_RBT_H_

#include "legacy_robot_types.hpp"
#include "armtarget.hpp"
#include "kinematic_error_info.hpp"
#include "math/pose.hpp"

// 2026-10-10 P2：ELBOW_TARGET_* 内部码族退役——统一码表见 basetarget.hpp。


namespace interpolation {

class CKinGeneRbt {
 private:
  void UpdateConfig_(CArmTarget& p, const MAN_ROBOT& mrp);
  void UpdateConfigError_(CArmTarget& p);
  void UpdateConfigNominal_(CArmTarget& p);

  bool CalcPose_(CArmTarget& p,
                 const MAN_ROBOT& mrp,
                 const FRAME& tool_frame,
                 bool robot_fixed_object);
  bool CalcPoseNominal_(CArmTarget& p,
                        const MAN_ROBOT& mrp,
                        const FRAME& tool_frame,
                        bool robot_fixed_object);

  int CalcJointNoConfigNominal_(CArmTarget& p,
                                const MAN_ROBOT& mrp,
                                const CArmTarget& old_rtp,
                                const FRAME& tool_frame,
                                bool robot_fixed_object,
                                bool use_old_config);
  bool CalcJointNoConfigError_(CArmTarget& new_t_p,
                               const MAN_ROBOT& mrp,
                               const CArmTarget& old_t_p,
                               const FRAME& tool_frame,
                               bool robot_fixed_object);

  bool CalcJointWithConfigError_(CArmTarget& new_t_p,
                                 const MAN_ROBOT& mrp,
                                 const CArmTarget& old_t_p,
                                 const FRAME& tool_frame,
                                 bool robot_fixed_object);
  int CalcJointWithConfigNominal_(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object);

  int CalcOrientation123NoConfig_(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object);
  bool CalcOrientation123NoConfigError_(CArmTarget& p,
                                        const MAN_ROBOT& mrp,
                                        const CArmTarget& old_rtp,
                                        const FRAME& tool_frame,
                                        bool robot_fixed_object);
  int CalcOrientation234NoConfig_(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object);

  bool CalcJointSingularityNominal_(CArmTarget& p,
                                    const MAN_ROBOT& mrp,
                                    const CArmTarget& old_rtp,
                                    const FRAME& tool_frame,
                                    bool robot_fixed_object,
                                    double tol,
                                    double weight,
                                    double errlim);

  int ElbowTargetKinJacobian_(math::Cartesian3<double>& pos,
                              math::Quaternion<double>& orient,
                              double* jac,
                              double* k,
                              double* j0rad,
                              int* re_use);

  static int SolveSinCosEquation_(double* t1, double* t2,
                                  double a, double b, double cc);

 public:
  CKinGeneRbt();
  ~CKinGeneRbt();

  int CalcPose(CArmTarget& p,
                const MAN_ROBOT& mrp,
                const FRAME& tool_frame,
                bool robot_fixed_object);
  int CalcPoseNominal(CArmTarget& p,
                       const MAN_ROBOT& mrp,
                       const FRAME& tool_frame,
                       bool robot_fixed_object);

  int CalcJointNoConfig(CArmTarget& p,
                        const MAN_ROBOT& mrp,
                        const CArmTarget& old_rtp,
                        const FRAME& tool_frame,
                        bool robot_fixed_object,
                        KinematicErrorInfo* err = nullptr);
  int CalcJointWithConfig(CArmTarget& p,
                           const MAN_ROBOT& mrp,
                           const CArmTarget& old_rtp,
                           const FRAME& tool_frame,
                           bool robot_fixed_object,
                           KinematicErrorInfo* err = nullptr);
  int CalcJointNoConfigNominal(CArmTarget& p,
                               const MAN_ROBOT& mrp,
                               const CArmTarget& old_rtp,
                               const FRAME& tool_frame,
                               bool robot_fixed_object,
                               bool use_old_config,
                               KinematicErrorInfo* err = nullptr);
  int CalcJointWithConfigNominal(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object,
                                  KinematicErrorInfo* err = nullptr);

  int CalcOrientation123NoConfig(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object);
  int CalcOrientation123NoConfigNominal(CArmTarget& p,
                                         const MAN_ROBOT& mrp,
                                         const CArmTarget& old_rtp,
                                         const FRAME& tool_frame,
                                         bool robot_fixed_object);
  int CalcOrientation234NoConfig(CArmTarget& p,
                                  const MAN_ROBOT& mrp,
                                  const CArmTarget& old_rtp,
                                  const FRAME& tool_frame,
                                  bool robot_fixed_object);

  bool CalcJointSingularity(CArmTarget& p,
                            const MAN_ROBOT& mrp,
                            const CArmTarget& old_rtp,
                            const FRAME& tool_frame,
                            bool robot_fixed_object,
                            double tol,
                            double weight,
                            double errlim);

  int CheckMasterConfig(const CArmTarget& tg1p,
                        const CArmTarget& tg2p,
                        bool check_wrist_only,
                        double rotation_limit);

  bool CalcVelocityJacobian(const CArmTarget& p,
                            const MAN_ROBOT& mrp,
                            std::array<std::array<double, 6>, 6>& jacobian);
  bool CalcVelocityJacobianNominal(const CArmTarget& p,
                                   const MAN_ROBOT& mrp,
                                   std::array<std::array<double, 6>, 6>& jacobian);

  bool CalcFakeTargetConfig(CArmTarget& fake_t_p,
                            CArmTarget& t_p,
                            const MAN_ROBOT& mrp,
                            double jacobian[36],
                            FRAME& tool,
                            bool robot_fixed_object);
  bool AddJ3AndCompliance(CArmTarget& p,
                          const MAN_ROBOT& mrp,
                          const MAN_ROBOT_JOINTS& joint_torques,
                          double jacobian[36],
                          math::Cartesian3<double>& robot_pos,
                          math::Quaternion<double>& robot_quat,
                          MAN_ROBOT_JOINTS& robot_joint_angles);
  bool RemoveJ3AndCompliance(CArmTarget& p,
                             const MAN_ROBOT& mrp,
                             const MAN_ROBOT_JOINTS& joint_torques,
                             double jacobian[36],
                             math::Cartesian3<double>& robot_pos,
                             math::Quaternion<double>& robot_quat,
                             MAN_ROBOT_JOINTS& robot_joint_angles);

  bool AddJ3(CArmTarget& p,
             const MAN_ROBOT& mrp,
             math::Cartesian3<double>& robot_pos,
             math::Quaternion<double>& robot_quat,
             MAN_ROBOT_JOINTS& robot_joint_angles);

  void GetCheckpoints(const CArmTarget& p,
                      math::Cartesian3<double>& cp_arm,
                      math::Cartesian3<double>& cp_wcp) const;
};

}  // namespace interpolation

#endif  // _ROB_KIN_GENE_RBT_H_
