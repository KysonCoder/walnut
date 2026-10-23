#ifndef BASE_CONFBASE_INCLUDE_SHM_LAYOUT_HPP_
#define BASE_CONFBASE_INCLUDE_SHM_LAYOUT_HPP_

#include "robot_config_common.hpp"

namespace robot_config {

constexpr uint32_t kMaxUnitJoints = 7;
constexpr uint32_t kMaxUnitLinks = 8;
constexpr uint32_t kMaxUnits = 15;
constexpr uint32_t kMaxNoOfJoints = 57;

constexpr uint32_t kMaxDynGroups = 42;

struct DynGroupConfig {
    int32_t dyngrp_model;
    int32_t unit_type;
    int32_t dyngrp_no;
    double  power_regain_limit;
    double  power_regain_min;
    double  emergency_stop_power_regain;
    int32_t reserved[14];
};
static_assert(sizeof(DynGroupConfig) == 96, "DynGroupConfig size mismatch");

struct DynGroupConfigPage {
    int32_t num_groups;
    int32_t reserved1;
    DynGroupConfig groups[kMaxDynGroups];
    char reserved2[kShmPageSize - sizeof(int32_t) * 2
                   - sizeof(DynGroupConfig) * kMaxDynGroups];
};
static_assert(sizeof(DynGroupConfigPage) == kShmPageSize,
              "DynGroupConfigPage size mismatch");

struct UnitEntry {
    uint32_t unit_type;
    uint64_t offset;
    uint64_t size;
    uint32_t page_index;
    uint32_t page_count;
    uint32_t num_joints;
    uint32_t num_links;
    uint64_t drivechain_offset;      // offset to DriveChainUnitData
    uint32_t drivechain_page_count;  // pages allocated for drivechain
    char reserved[4];                // padding for alignment
};

struct ShmHeader {
    uint64_t magic;
    uint32_t version;
    uint32_t crc32;
    uint32_t page_size;
    uint32_t num_units;
    uint64_t total_size;
    UnitEntry entries[kMaxUnits];
    uint64_t urdf_offset;
    uint64_t urdf_size;
    uint64_t dyngrp_offset;
    uint64_t dyngrp_size;
    uint64_t topology_offset;   // NEW
    uint64_t topology_size;     // NEW
    char reserved[992];         // was 1008, reduced by 16
};
static_assert(sizeof(ShmHeader) <= kShmPageSize, "ShmHeader too large");

struct GlobalJointMap {
    int32_t logical_to_physical[kMaxNoOfJoints];
    int32_t physical_to_logical[kMaxNoOfJoints];
    char reserved[kShmPageSize - (kMaxNoOfJoints * 4 * 2)];
};
static_assert(sizeof(GlobalJointMap) == kShmPageSize, "GlobalJointMap size");

struct GlobalJointParams {
    double gear_ratio[kMaxNoOfJoints];
    double joint_zero[kMaxNoOfJoints];
    double encoder_resolution[kMaxNoOfJoints];
    char reserved[kShmPageSize - (kMaxNoOfJoints * 8 * 3)];
};
static_assert(sizeof(GlobalJointParams) == kShmPageSize, "GlobalJointParams size");

constexpr uint32_t kMaxChains = 8;

struct KinematicChainConfig {
    char name[64];
    int32_t chain_type;
    int32_t num_units;
    int32_t unit_types[kMaxUnitJoints];
    char chain_tip_link[64];
    int32_t reserved[16];
};

struct RobotTopology {
    int32_t num_chains;
    char global_base_link[64];
    KinematicChainConfig chains[kMaxChains];
    char reserved[2204];
};

static_assert(sizeof(RobotTopology) == kShmPageSize, "RobotTopology size mismatch");

struct UnitConfig {
    int32_t unit_type;
    int32_t num_joints;
    int32_t num_links;
    int32_t reserved1;
    JointDriveParams joints_drive[kMaxUnitJoints];
    JointKinematicParams joints_kinematic[kMaxUnitJoints];
    LinkDynamicParams links_dynamic[kMaxUnitLinks];
    double linear_max_speed;
    double angular_max_speed;
    double teach_mode_max_speed;
    double speed_control_percent;
    double tcp_transform[16];
    double tcp_mass;
    double tcp_com[3];
    int32_t logical_to_physical[kMaxUnitJoints];
    int32_t physical_to_logical[kMaxUnitJoints];
    char unit_name[kNameLength];              // 32
    char kinematic_model_name[64];            // NEW
    char root_link[64];                       // NEW
    char tip_link[64];                        // NEW
    char reserved2[64];                       // was 256
};
static_assert(sizeof(UnitConfig) <= kShmPageSize, "UnitConfig too large");

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_SHM_LAYOUT_HPP_
