#ifndef BASE_CONFBASE_INCLUDE_HUMANOID_CONFIG_STORE_HPP_
#define BASE_CONFBASE_INCLUDE_HUMANOID_CONFIG_STORE_HPP_

#include <cstdint>

#include "humanoid_conf.hpp"
#include "shm_layout.hpp"
#include "drivechain/drive_chain_tables.hpp"
#include "robot_config_serializer.hpp"

namespace robot_config {

class HumanoidConfigStore {
 public:
  static HumanoidConfigStore& Instance();

  ConfigError Attach(const char* mmap_path);
  ConfigError Attach(const void* base, size_t size);
  void Detach();
  bool IsAttached() const;

  const HumanoidConfigLayout* GetLayout() const;
  const HumanoidShmHeader* GetHeader() const;
  const HumanoidGlobalJointMap* GetGlobalJointMap() const;
  const HumanoidGlobalJointParams* GetGlobalJointParams() const;
  const HumanoidUnitConfig* GetUnit(HumanoidSocUnitType unit_type) const;
  const drivechain::DriveChainUnitData* GetDriveChainData(
      HumanoidSocUnitType unit_type) const;

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
  HumanoidConfigStore() = default;
  ~HumanoidConfigStore();

  internal::RobotConfigSerializer serializer_;
  const HumanoidConfigLayout* layout_ = nullptr;
};

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_HUMANOID_CONFIG_STORE_HPP_
