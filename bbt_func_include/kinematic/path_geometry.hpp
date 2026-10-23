///@brief Path geometry computation class
///
/// Pure math operations for path interpolation, distance calculation,
/// and circle initialization. All methods are static and operate on
/// math types (Cartesian3, Quaternion, joint arrays) without
/// dependency on robot-specific kinematics.
///
///@author Yonglin
///@version 0.1
///@date 2026-05-05

#ifndef _PATH_GEOMETRY_H_
#define _PATH_GEOMETRY_H_

#include "confbase/humanoid_conf.hpp"
#include "legacy_robot_types.hpp"
#include "quaternion.hpp"
#include "pose.hpp"

///@name Error codes used by PathGeometry
///@{

/// Quaternion is not normalized (warning)
#define ROBOT_TARGET_QUATERNION_NOT_NORMALIZED                  20

/// Circle input points are too close to each other
#define ROBOT_TARGET_CIRCLE_POINTS_TOO_CLOSE                   -10
/// Three circle points are collinear
#define ROBOT_TARGET_CIRCLE_COLLAPSED                          -11
/// Circle definition is ambiguous and must be selected
#define ROBOT_TARGET_SELECT_CIRCLE_DEFINITION                  -12
/// Circle end point is too close to start point
#define ROBOT_TARGET_CIRCLE_END_POINT_TOO_CLOSE_TO_START       -21
/// Circle intermediate point is too close to start point
#define ROBOT_TARGET_CIRCLE_POINT_TOO_CLOSE_TO_START           -22

/// Cartesian vector has zero length
#define MATH_LIB_CARTESIAN_NULL_VECTOR -1

///@}

namespace interpolation {

///@brief Interpolation method for joint subset selection.
///
/// Determines which joint axes are interpolated or blended.
enum path_ipol_method {
  path_line,       ///< Linear cartesian interpolation (no joints)
  path_joint_123,  ///< Interpolate joints 1, 2, 3
  path_joint_456,  ///< Interpolate joints 4, 5, 6
  path_joint_156,  ///< Interpolate joints 1, 5, 6
  path_joint_134,  ///< Interpolate joints 1, 3, 4
  path_joint_567   ///< Interpolate joints 5, 6, 7
};

///@brief Static class for path geometry computations.
///
/// All methods operate on pure math types. No robot model is required.
class PathGeometry {
 public:
  ///@name Distance
  ///@{

  ///@brief Compute the dot product (p1 - p2) . (p3 - p2).
  ///@param p1 First position.
  ///@param p2 Second (center) position.
  ///@param p3 Third position.
  ///@return Scalar dot product.
  static double CartesianDotProd(const math::Cartesian3<double>& p1,
                                 const math::Cartesian3<double>& p2,
                                 const math::Cartesian3<double>& p3);

  ///@brief Euclidean distance between two cartesian points.
  ///@param p1 Start point.
  ///@param p2 End point.
  ///@return Straight-line distance.
  static double CalcLineDist(const math::Cartesian3<double>& p1,
                             const math::Cartesian3<double>& p2);

  ///@brief Rotation angle and axis between two quaternions.
  ///@param q_start Start orientation.
  ///@param q_end   End orientation.
  ///@param axis_out Output rotation axis (unit vector).
  ///@return Rotation angle in radians [0, pi].
  static double CalcQuaternionDist(const math::Quaternion<double>& q_start,
                                   const math::Quaternion<double>& q_end,
                                   math::Cartesian3<double>& axis_out);

  ///@brief Absolute joint angle differences.
  ///@param joints1 First joint set.
  ///@param joints2 Second joint set.
  ///@param abs_joint_dist Output array of absolute differences.
  ///@param no_of_joints Number of joints to compare.
  static void CalcJointsAbsDist(const SOC_JOINTS& joints1,
                                const SOC_JOINTS& joints2,
                                double abs_joint_dist[],
                                int no_of_joints);

  ///@brief Maximum linear and rotational joint distances.
  ///@param joints1 First joint set.
  ///@param joints2 Second joint set.
  ///@param linear_dist    Output maximum linear joint distance.
  ///@param rotational_dist Output maximum rotational joint distance.
  ///@param robot_param    Robot parameters defining joint types.
  static void CalcJointsDist(const SOC_JOINTS& joints1,
                             const SOC_JOINTS& joints2,
                             double& linear_dist,
                             double& rotational_dist,
                             const MAN_ROBOT& robot_param);

  ///@}

  ///@name Interpolation
  ///@{

  ///@brief Linear cartesian interpolation.
  ///@param start Start position (L = 1).
  ///@param end   End position (L = 0).
  ///@param L     Interpolation parameter [0, 1].
  ///@return Interpolated position.
  static math::Cartesian3<double> IpolLine(
      const math::Cartesian3<double>& start,
      const math::Cartesian3<double>& end, double L);

  ///@brief Arm angle interpolation for 7-DOF arms.
  ///@param start_angle Start arm angle (L = 1).
  ///@param end_angle   End arm angle (L = 0).
  ///@param L           Interpolation parameter [0, 1].
  ///@return Interpolated arm angle.
  ///@note No-op when _7_JOINTS_ARM is not defined.
  static double IpolArmAngle(double start_angle, double end_angle, double L);

  ///@brief Joint interpolation over all active joints.
  ///@param out         Output joint array.
  ///@param start       Start joints (L = 1).
  ///@param end         End joints (L = 0).
  ///@param no_of_joints Number of joints to interpolate.
  ///@param L           Interpolation parameter [0, 1].
  static void IpolJoints(SOC_JOINTS& out,
                         const SOC_JOINTS& start,
                         const SOC_JOINTS& end,
                         int no_of_joints, double L);

  ///@brief Joint interpolation for a specific joint subset.
  ///@param out    Output joint array.
  ///@param start  Start joints (L = 1).
  ///@param end    End joints (L = 0).
  ///@param L      Interpolation parameter [0, 1].
  ///@param method Which joint subset to interpolate.
  static void IpolJointsSubset(SOC_JOINTS& out,
                               const SOC_JOINTS& start,
                               const SOC_JOINTS& end,
                               double L, path_ipol_method method);

  ///@brief Quaternion interpolation around a fixed axis.
  ///@param start_orient Start orientation.
  ///@param L            Interpolation parameter [0, 1].
  ///@param fi           Total rotation angle.
  ///@param axis         Rotation axis (unit vector).
  ///@return Interpolated quaternion.
  static math::Quaternion<double> IpolQuaternion(
      const math::Quaternion<double>& start_orient,
      double L, double fi, const math::Cartesian3<double>& axis);

  ///@brief Circular arc position interpolation.
  ///@param L                  Interpolation parameter [0, 1].
  ///@param circle_radius      Radius of the circle.
  ///@param opening_angle      Opening angle of the arc.
  ///@param start_angle        Start angle on the circle.
  ///@param circle_centre      Circle center in object frame.
  ///@param q_circle_rel_object Orientation of circle frame relative to object.
  ///@param arc_dir_out        Output tangent direction at interpolated point.
  ///@return Interpolated position on the arc.
  static math::Cartesian3<double> IpolCircle(
      double L, double circle_radius, double opening_angle,
      double start_angle, const math::Cartesian3<double>& circle_centre,
      const math::Quaternion<double>& q_circle_rel_object,
      math::Cartesian3<double>& arc_dir_out);

  ///@brief Circular arc orientation interpolation.
  ///@param L                          Interpolation parameter [0, 1].
  ///@param opening_angle              Arc opening angle.
  ///@param start_angle                Start angle on circle.
  ///@param axis_start_end_rel_circle  Rotation axis in circle frame.
  ///@param theta_start_end_rel_circle Rotation angle from start to end.
  ///@param q_circle_rel_object        Circle frame orientation.
  ///@param q_start_rel_arc            Start orientation relative to arc frame.
  ///@return Interpolated orientation.
  static math::Quaternion<double> IpolCircleQuat(
      double L, double opening_angle, double start_angle,
      const math::Cartesian3<double>& axis_start_end_rel_circle,
      double theta_start_end_rel_circle,
      const math::Quaternion<double>& q_circle_rel_object,
      const math::Quaternion<double>& q_start_rel_arc);

  ///@brief Corner quaternion interpolation via a corner orientation.
  ///@param corner_orient Corner orientation.
  ///@param L             Interpolation parameter [0, 1].
  ///@param fi1           Rotation angle from start to corner.
  ///@param axis1         Rotation axis from start to corner.
  ///@param fi2           Rotation angle from corner to end.
  ///@param axis2         Rotation axis from corner to end.
  ///@return Interpolated quaternion.
  static math::Quaternion<double> IpolCornerQuat(
      const math::Quaternion<double>& corner_orient,
      double L, double fi1, const math::Cartesian3<double>& axis1,
      double fi2, const math::Cartesian3<double>& axis2);

  ///@}

  ///@name Circle initialization
  ///@{

  ///@brief Initialize a circle from three points.
  ///@param p_start              Start point.
  ///@param p_circle             Intermediate point defining circle shape.
  ///@param p_end                End point.
  ///@param circle_radius        Output circle radius.
  ///@param opening_angle        Output arc opening angle.
  ///@param start_angle          Output start angle on circle.
  ///@param circle_centre        Output circle center.
  ///@param q_circle_rel_object  Output circle frame orientation.
  ///@return true on success, error code on failure.
  static int InitCircle(
      const math::Cartesian3<double>& p_start,
      const math::Cartesian3<double>& p_circle,
      const math::Cartesian3<double>& p_end,
      double& circle_radius, double& opening_angle, double& start_angle,
      math::Cartesian3<double>& circle_centre,
      math::Quaternion<double>& q_circle_rel_object);

  ///@brief Initialize circle orientation interpolation.
  ///
  /// Computes start/end orientations relative to the circle arc frame.
  ///@param q_start                  Start orientation.
  ///@param q_end                    End orientation.
  ///@param opening_angle            Arc opening angle.
  ///@param start_angle              Start angle on circle.
  ///@param q_circle_rel_object      Circle frame orientation.
  ///@param q_start_rel_circle_arc   Output start orientation in arc frame.
  ///@param q_end_rel_circle_arc     Output end orientation in arc frame.
  ///@param axis_start_end_rel_circle Output rotation axis in circle frame.
  ///@param theta_start_end_rel_circle Output rotation angle.
  ///@return true on success.
  static bool InitCircleQuat(
      const math::Quaternion<double>& q_start,
      const math::Quaternion<double>& q_end,
      double opening_angle, double start_angle,
      const math::Quaternion<double>& q_circle_rel_object,
      math::Quaternion<double>& q_start_rel_circle_arc,
      math::Quaternion<double>& q_end_rel_circle_arc,
      math::Cartesian3<double>& axis_start_end_rel_circle,
      double& theta_start_end_rel_circle);

  ///@brief Select the correct circle quaternion rotation direction.
  ///@param q_circle                     Circle frame orientation.
  ///@param q_start                      Start orientation.
  ///@param q_end                        End orientation.
  ///@param circle_radius                Circle radius.
  ///@param start_angle                  Start angle.
  ///@param opening_angle                Opening angle.
  ///@param axis_start_end_rel_circle    Rotation axis.
  ///@param theta_start_end_rel_circle_in Output rotation angle (may be negated).
  ///@return 0 on success.
  static int SelectCircleQuat(
      const math::Quaternion<double>& q_circle,
      const math::Quaternion<double>& q_start,
      const math::Quaternion<double>& q_end,
      double circle_radius, double start_angle, double opening_angle,
      const math::Cartesian3<double>& axis_start_end_rel_circle,
      double& theta_start_end_rel_circle_in);

  ///@}

  ///@name Blend (Add/Sub)
  ///@{

  ///@brief Blend positions: p1 + p2 - p_c.
  ///@param p1  First position.
  ///@param p2  Second position.
  ///@param p_c Center position to subtract.
  ///@return Blended position.
  static math::Cartesian3<double> AddSubPosition(
      const math::Cartesian3<double>& p1,
      const math::Cartesian3<double>& p2,
      const math::Cartesian3<double>& p_c);

  ///@brief Blend arm angles: a1 + a2 - a_c.
  ///@param p1_angle First arm angle.
  ///@param p2_angle Second arm angle.
  ///@param p_c_angle Center arm angle to subtract.
  ///@return Blended arm angle.
  ///@note No-op when _7_JOINTS_ARM is not defined.
  static double AddSubArmAngle(double p1_angle, double p2_angle,
                               double p_c_angle);

  ///@brief Blend joints: j1 + j2 - j_c.
  ///@param out        Output joint array.
  ///@param p1         First joint set.
  ///@param p2         Second joint set.
  ///@param p_c        Center joint set to subtract.
  ///@param no_of_joints Number of joints.
  static void AddSubJoints(SOC_JOINTS& out,
                           const SOC_JOINTS& p1,
                           const SOC_JOINTS& p2,
                           const SOC_JOINTS& p_c,
                           int no_of_joints);

  ///@brief Blend a specific joint subset.
  ///@param out    Output joint array.
  ///@param p1     First joint set.
  ///@param p2     Second joint set.
  ///@param p_c    Center joint set to subtract.
  ///@param method Which joint subset to blend.
  static void AddSubJointsSubset(SOC_JOINTS& out,
                                 const SOC_JOINTS& p1,
                                 const SOC_JOINTS& p2,
                                 const SOC_JOINTS& p_c,
                                 path_ipol_method method);

  ///@}

  ///@name Utility
  ///@{

  ///@brief Return a copy of a quaternion.
  ///@param orient Quaternion to copy.
  ///@return Identical quaternion.
  static math::Quaternion<double> CopyOrient(
      const math::Quaternion<double>& orient);

  ///@}
};

}  // namespace interpolation

#endif
