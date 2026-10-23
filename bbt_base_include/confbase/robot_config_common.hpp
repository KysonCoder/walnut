#ifndef BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_COMMON_HPP_
#define BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_COMMON_HPP_

#include <cstdint>

namespace robot_config {

constexpr uint64_t kShmMagic = 0x524F424F54434647ULL;  // "ROBOTCFG"
constexpr uint32_t kShmVersion = 5;  // bumped: +kinematic_model_name/root_link/tip_link + RobotTopology
constexpr uint32_t kShmPageSize = 4096;
constexpr uint32_t kNameLength = 32;

enum class ConfigError : int32_t {
  OK = 0,
  FILE_NOT_FOUND = -1,
  PARSE_ERROR = -2,
  MISSING_SECTION = -3,
  MAGIC_MISMATCH = -4,
  VERSION_MISMATCH = -5,
  CRC_MISMATCH = -6,
  PAGE_SIZE_MISMATCH = -7,
  ALREADY_ATTACHED = -8,
  UNIT_NOT_FOUND = -9,
  BUILD_FAILED = -10,
  MMAP_FAILED = -11,
  ROBOT_TYPE_MISMATCH = -12,
  INVALID_PARAM = -13,
};

enum class KinematicType : int32_t {
  STANDARD_DH = 0,
  MODIFIED_DH = 1,
  URDF_POSE = 2,
};

constexpr uint32_t KINEMATIC_FLAG_SDH = 0x01;
constexpr uint32_t KINEMATIC_FLAG_MDH = 0x02;
constexpr uint32_t KINEMATIC_FLAG_URDF = 0x04;

struct JointDriveParams {
  double gear_ratio;
  double encoder_resolution;
  double joint_zero;
  double pos_positive_limit;
  double pos_negative_limit;
  double speed_positive_limit;
  double speed_negative_limit;
  double acc_limit;
  double dec_limit;
  double motor_inertia;
  double gear_inertia;
  int32_t servo_id;
  int32_t logical_joint_index;
  int32_t physical_joint_index;
  int32_t reserved;
};
static_assert(sizeof(JointDriveParams) == 104,
              "JointDriveParams size mismatch");

struct JointKinematicParams {
  struct { double theta, d, a, alpha; } sdh;
  struct { double alpha, a, theta, d; } mdh;
  struct {
    double xyz[3];
    double rpy[3];
    double axis[3];
  } urdf;
  double joint_limit_min;
  double joint_limit_max;
  double velocity_limit;
  double acceleration_limit;
  int32_t joint_type;
  int32_t axis_direction;
  uint32_t flags;
  char name[kNameLength];
};
static_assert(sizeof(JointKinematicParams) == 216,
              "JointKinematicParams size mismatch");

struct LinkDynamicParams {
  double mass;
  double center_of_mass[3];
  double inertia_tensor[9];
  double damping;
  double coulomb_friction;
  double viscous_friction;
  int32_t parent_link_index;
  int32_t reserved;
};
static_assert(sizeof(LinkDynamicParams) == 136,
              "LinkDynamicParams size mismatch");

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_COMMON_HPP_
