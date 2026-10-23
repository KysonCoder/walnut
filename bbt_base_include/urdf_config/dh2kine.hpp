///@brief Convert modified DH parameters to internal kinematic parameters
///@author Yonglin
///@version 0.1
///@date 18-Apr-2026

#ifndef _DH2KINE_H_
#define _DH2KINE_H_

#include <array>
#include "confbase/humanoid_conf.hpp"

namespace interpolation {

/// Modified Denavit-Hartenberg parameters for a standard 6-DOF elbow robot
struct MDHParams {
  std::array<double, 6> a;      ///< Link lengths (a_i)
  std::array<double, 6> alpha;  ///< Link twists (alpha_i, rad)
  std::array<double, 6> d;      ///< Link offsets (d_i)
  std::array<double, 6> theta;  ///< Joint angle offsets (theta_i, rad)
};

/// Conversion utility from modified DH parameters to internal kinematic parameters
class CDH2Kine {
 public:
  /// Convert mDH to CKinPLinkRbt-compatible MAN_ROBOT (standard elbow model)
  /// @param mdh Modified DH parameters
  /// @param offset_of_upper_arm Additional Z offset at joint 3 (default: 0)
  /// @param cp_body_c Arm checkpoint offset in joint-3 frame
  static MAN_ROBOT ToPLinkRbt(
      const MDHParams& mdh,
      double offset_of_upper_arm = 0.0,
      const math::Cartesian3<double>& cp_body_c = math::Cartesian3<double>{
          -0.25, -0.20, 0.25});

  /// Convert mDH to CKinGeneRbt-compatible MAN_ROBOT (decomposed upper arm)
  /// The upper arm is decomposed into horizontal (housing+tube) and vertical
  /// (offset) components. These must be measured from the actual mechanical
  /// structure; they cannot be derived from mDH alone.
  /// @param mdh Modified DH parameters
  /// @param offset_of_upper_arm Vertical offset component at joint 3
  /// @param length_of_housing_axis_4 Horizontal housing component
  /// @param length_of_tube_shaft Horizontal tube component
  /// @param cp_body_c Arm checkpoint offset in joint-3 frame
  static MAN_ROBOT ToGeneRbt(
      const MDHParams& mdh,
      double offset_of_upper_arm,
      double length_of_housing_axis_4,
      double length_of_tube_shaft,
      const math::Cartesian3<double>& cp_body_c = math::Cartesian3<double>{
          -0.12, -0.10, 0.12});

  // ---- Preset mDH parameters for known robots ----

  /// ABB IRB140 modified DH parameters (Craig convention)
  static MDHParams MakeIRB140MDH();

  /// ABB IRB2400 modified DH parameters (Craig convention)
  static MDHParams MakeIRB2400MDH();

  // ---- Convenience factories for known robots ----

  /// Pre-configured IRB2400 for CKinPLinkRbt (verified against legacy data)
  static MAN_ROBOT MakeIRB2400_PLinkRbt();

  /// Pre-configured IRB140 for CKinPLinkRbt
  static MAN_ROBOT MakeIRB140_PLinkRbt();

  /// Pre-configured IRB140 for CKinGeneRbt (verified against test suite)
  static MAN_ROBOT MakeIRB140_GeneRbt();

  /// Pre-configured IRB2400 for CKinGeneRbt (approximated decomposition)
  static MAN_ROBOT MakeIRB2400_GeneRbt();
};

}  // namespace interpolation

#endif  // _DH2KINE_H_
