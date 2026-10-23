#ifndef BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TYPES_HPP_
#define BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TYPES_HPP_

#include <cstdint>

namespace drivechain {

enum class DriveChainStatus : int {
  kOk = 0,
  kInvalidParam = -1,
  kConfigError = -2,
  kTableOverflow = -3,
  kNullPointer = -4,
  kBuildFailed = -5
};

enum class DriveChainMode : uint8_t {
  kNormal = 0,
  kOptimalStop = 1,
  kEmergencyStop = 2
};

enum class EmergencyPhase : uint8_t {
  kPhase1 = 0,
  kPhase2 = 1,
  kPhase3 = 2
};

}  // namespace drivechain

#endif  // BASE_DRIVECHAIN_INCLUDE_DRIVE_CHAIN_TYPES_HPP_
