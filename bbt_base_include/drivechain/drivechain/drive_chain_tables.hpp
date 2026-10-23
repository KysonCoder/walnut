#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TABLES_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TABLES_HPP_

#include <cstdint>
#include "drivechain/drive_chain_config.hpp"

namespace drivechain {

constexpr int kDriveChainTableSize = 64;
constexpr uint32_t kDriveChainVersion = 1;

struct DriveChainTablePoint {
  float m;
  float k;
};

struct DriveChainJointTables {
  DriveChainTablePoint normal_acc[kDriveChainTableSize];
  DriveChainTablePoint normal_dec[kDriveChainTableSize];
  DriveChainTablePoint optimal_acc[kDriveChainTableSize];
  DriveChainTablePoint optimal_dec[kDriveChainTableSize];
  DriveChainTablePoint em_stop_p1[kDriveChainTableSize];
  DriveChainTablePoint em_stop_p2[kDriveChainTableSize];
  DriveChainTablePoint em_stop_p3[kDriveChainTableSize];
  float table_max_speed;
  float table_delta_speed_inv;
  float max_speed;
};

// Per-unit drivechain data stored after UnitConfig in shared memory
struct DriveChainUnitData {
  DriveChainJointConfig joint_configs[7];  // kMaxUnitJoints = 7
  DriveChainJointTables joint_tables[7];
  uint32_t num_joints;
  uint32_t version;  // kDriveChainVersion
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TABLES_HPP_
