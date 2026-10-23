#ifndef BASE_CONFBASE_INCLUDE_DUALARM_CONF_HPP_
#define BASE_CONFBASE_INCLUDE_DUALARM_CONF_HPP_

#include "robot_config_common.hpp"
#include "shm_layout.hpp"

namespace robot_config {

constexpr uint32_t kDualArmMaxUnits = 3;
constexpr uint32_t kDualArmMaxJoints = 13;
constexpr uint32_t kDualArmMaxUnitJoints = 7;
constexpr uint32_t kDualArmMaxUnitLinks = 8;

enum class DualArmSocUnitType : int32_t {
  NO_UNIT = -1,
  RIGHT_ARM = 0,
  LEFT_ARM = 1,
  ROBOT_BODY = 2,
};

enum class DualArmTargetType : int32_t {
  NO_TARGET = -1,
  HEAD_TARGET = 0,
  ARM_TARGET = 1,
  THUMB_TARGET = 2,
  GFINGER_TARGET = 3,
  LITTLE_FINGER_TARGET = 4,
  ROBOT_BODY_TARGET = 5,
  AGV_FOOT_TARGET = 6,
};

struct DualArmUnitConfig {
  int32_t unit_type;
  int32_t num_joints;
  int32_t num_links;
  int32_t reserved1;
  JointDriveParams joints_drive[kDualArmMaxUnitJoints];
  JointKinematicParams joints_kinematic[kDualArmMaxUnitJoints];
  LinkDynamicParams links_dynamic[kDualArmMaxUnitLinks];
  double linear_max_speed;
  double angular_max_speed;
  double teach_mode_max_speed;
  double speed_control_percent;
  double tcp_transform[16];
  double tcp_mass;
  double tcp_com[3];
  int32_t logical_to_physical[kDualArmMaxUnitJoints];
  int32_t physical_to_logical[kDualArmMaxUnitJoints];
  char unit_name[kNameLength];
  char kinematic_model_name[64];
  char root_link[64];
  char tip_link[64];
  // Pad to exactly kShmPageSize (4096 - sum of all preceding fields).
  char reserved2[280];
};
static_assert(sizeof(DualArmUnitConfig) == kShmPageSize,
              "DualArmUnitConfig must fill exactly one page");

// Global joint map/params follow the 57-wide generic contract (shm_layout.hpp)
// shared by both flavors: CJointPos and all generic consumers read this layout
// directly. The dualarm type names are kept as aliases; the logical joint
// count remains kDualArmMaxJoints (13) — entries beyond the logical axes stay
// zero-filled.
using DualArmGlobalJointMap = GlobalJointMap;
using DualArmGlobalJointParams = GlobalJointParams;

struct DualArmShmHeader {
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
  } entries[kDualArmMaxUnits];

  uint64_t urdf_offset;
  uint64_t urdf_size;
  uint64_t dyngrp_offset;
  uint64_t dyngrp_size;
  uint64_t topology_offset;
  uint64_t topology_size;
  // Pad to exactly kShmPageSize (4096 - sum of all preceding fields).
  char reserved[3848];
};
static_assert(sizeof(DualArmShmHeader) == kShmPageSize,
              "DualArmShmHeader must fit exactly one page");

struct DualArmConfigLayout {
  DualArmShmHeader header;
  DualArmGlobalJointMap global_map;
  DualArmGlobalJointParams global_params;
  DualArmUnitConfig units[kDualArmMaxUnits];
};
static_assert(sizeof(DualArmConfigLayout) ==
              kShmPageSize * (3 + kDualArmMaxUnits),
              "DualArmConfigLayout size mismatch");

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_DUALARM_CONF_HPP_
