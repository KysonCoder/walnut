#ifndef BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_STORE_HPP_
#define BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_STORE_HPP_

#include <cstdint>

#include "dualarm_conf.hpp"
#include "shm_layout.hpp"
#include "drivechain/drive_chain_tables.hpp"
#include "robot_config_serializer.hpp"

namespace robot_config {

class DualArmConfigStore {
 public:
  static DualArmConfigStore& Instance();

  ConfigError Attach(const char* mmap_path);
  ConfigError Attach(const void* base, size_t size);
  void Detach();
  bool IsAttached() const;

  const DualArmConfigLayout* GetLayout() const;
  const DualArmShmHeader* GetHeader() const;
  const DualArmGlobalJointMap* GetGlobalJointMap() const;
  const DualArmGlobalJointParams* GetGlobalJointParams() const;
  const DualArmUnitConfig* GetUnit(DualArmSocUnitType unit_type) const;
  const drivechain::DriveChainUnitData* GetDriveChainData(
      DualArmSocUnitType unit_type) const;
  const char* GetUrdfString() const;
  const RobotTopology* GetRobotTopology() const;

  const DynGroupConfigPage* GetDynGroupConfigPage() const;
  const DynGroupConfig* FindDynGroupConfig(int32_t unit_type,
                                           int32_t dyngrp_model) const;
  bool GetDynGroupNumber(int32_t unit_type, int32_t dyngrp_model,
                         int32_t* out_dyngrp_no) const;
  bool GetPowerRegainLimit(int32_t unit_type, int32_t dyngrp_model,
                           double* out_limit) const;

 private:
  DualArmConfigStore() = default;
  ~DualArmConfigStore();

  internal::RobotConfigSerializer serializer_;
  const DualArmConfigLayout* layout_ = nullptr;
};

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_STORE_HPP_
