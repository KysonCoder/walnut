#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_PARSER_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_PARSER_HPP_

#include "drivechain/drive_chain_config.hpp"

namespace drivechain {

class DriveChainCfgParser {
 public:
  bool ParseJointFromFile(const char* file_path, const char* joint_name,
                          DriveChainJointConfig* out_config);
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_PARSER_HPP_
