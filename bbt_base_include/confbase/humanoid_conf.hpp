#ifndef BASE_CONFBASE_INCLUDE_HUMANOID_CONF_HPP_
#define BASE_CONFBASE_INCLUDE_HUMANOID_CONF_HPP_

#include "robot_config_common.hpp"
#include "shm_layout.hpp"

namespace robot_config {

constexpr uint32_t kHumanoidMaxUnits = 15;
constexpr uint32_t kHumanoidMaxJoints = 57;
constexpr uint32_t kHumanoidMaxUnitJoints = 7;
constexpr uint32_t kHumanoidMaxUnitLinks = 8;

enum class HumanoidSocUnitType : int32_t {
  NO_UNIT = -1,
  RIGHT_ARM = 0,
  RIGHT_THUMB = 1,
  RIGHT_FF = 2,
  RIGHT_MF = 3,
  RIGHT_RF = 4,
  RIGHT_LF = 5,
  LEFT_ARM = 6,
  LEFT_THUMB = 7,
  LEFT_FF = 8,
  LEFT_MF = 9,
  LEFT_RF = 10,
  LEFT_LF = 11,
  ROBOT_HEAD = 12,
  ROBOT_BODY = 13,
  AGV_FOOT = 14,
};

enum class HumanoidTargetType : int32_t {
  NO_TARGET = -1,
  HEAD_TARGET = 0,
  ARM_TARGET = 1,
  THUMB_TARGET = 2,
  GFINGER_TARGET = 3,
  LITTLE_FINGER_TARGET = 4,
  ROBOT_BODY_TARGET = 5,
  AGV_FOOT_TARGET = 6,
};

struct HumanoidUnitConfig {
  int32_t unit_type;
  int32_t num_joints;
  int32_t num_links;
  int32_t reserved1;
  JointDriveParams joints_drive[kHumanoidMaxUnitJoints];
  JointKinematicParams joints_kinematic[kHumanoidMaxUnitJoints];
  LinkDynamicParams links_dynamic[kHumanoidMaxUnitLinks];
  double linear_max_speed;
  double angular_max_speed;
  double teach_mode_max_speed;
  double speed_control_percent;
  double tcp_transform[16];
  double tcp_mass;
  double tcp_com[3];
  int32_t logical_to_physical[kHumanoidMaxUnitJoints];
  int32_t physical_to_logical[kHumanoidMaxUnitJoints];
  char unit_name[kNameLength];
  char kinematic_model_name[64];
  char root_link[64];
  char tip_link[64];
  // Pad to exactly kShmPageSize (4096 - sum of all preceding fields).
  char reserved2[280];
};
static_assert(sizeof(HumanoidUnitConfig) == kShmPageSize,
              "HumanoidUnitConfig must fill exactly one page");

// Global joint map/params follow the 57-wide generic contract (shm_layout.hpp)
// shared by both flavors: CJointPos and all generic consumers read this layout
// directly. The humanoid type names are kept as aliases; kHumanoidMaxJoints
// equals the generic kMaxNoOfJoints (57).
using HumanoidGlobalJointMap = GlobalJointMap;
using HumanoidGlobalJointParams = GlobalJointParams;

struct HumanoidShmHeader {
  uint64_t magic;
  uint32_t version;
  uint32_t crc32;
  uint32_t page_size;
  uint32_t num_units;
  uint64_t total_size;

  struct UnitEntry {
    uint32_t unit_type;
    uint64_t offset;
    uint64_t size;
    uint32_t page_index;
    uint32_t page_count;
    uint32_t num_joints;
    uint32_t num_links;
    uint64_t drivechain_offset;
    uint32_t drivechain_page_count;
    char reserved[4];
  } entries[kHumanoidMaxUnits];

  uint64_t urdf_offset;
  uint64_t urdf_size;
  uint64_t dyngrp_offset;
  uint64_t dyngrp_size;
  uint64_t topology_offset;
  uint64_t topology_size;
  // Pad to exactly kShmPageSize (4096 - sum of all preceding fields).
  char reserved[3176];  // Pad to exactly one page.
};
static_assert(sizeof(HumanoidShmHeader) == kShmPageSize,
              "HumanoidShmHeader must fit exactly one page");

struct HumanoidConfigLayout {
  HumanoidShmHeader header;
  HumanoidGlobalJointMap global_map;
  HumanoidGlobalJointParams global_params;
  HumanoidUnitConfig units[kHumanoidMaxUnits];
};
static_assert(sizeof(HumanoidConfigLayout) ==
              kShmPageSize * (3 + kHumanoidMaxUnits),
              "HumanoidConfigLayout size mismatch");

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_HUMANOID_CONF_HPP_
