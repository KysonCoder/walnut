#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_BUILDER_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_BUILDER_HPP_

#include "drivechain/drive_chain_config.hpp"
#include "drivechain/drive_chain_tables.hpp"

namespace drivechain {

class DriveChainBuilder {
 public:
  explicit DriveChainBuilder(DriveChainJointConfig* config_array,
                             DriveChainJointTables* tables_array,
                             int joint_count);

  bool SetJointConfig(int joint_idx, const DriveChainJointConfig& config);
  bool BuildAllTables();

 private:
  DriveChainJointConfig* config_array_;
  DriveChainJointTables* tables_array_;
  int joint_count_;
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_BUILDER_HPP_
