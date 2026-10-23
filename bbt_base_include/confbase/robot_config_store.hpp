#ifndef BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_STORE_HPP_
#define BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_STORE_HPP_

#include "shm_layout.hpp"
#include "drivechain/drive_chain_tables.hpp"
#include <cstddef>

namespace robot_config {

class RobotConfigStore {
 public:
  static RobotConfigStore& Instance();

  bool Attach(const char* mmap_path);
  bool Attach(const void* base, size_t size);
  void Detach();
  bool Validate() const;

  bool IsAttached() const;
  uint32_t GetVersion() const;
  uint32_t GetNumUnits() const;
  const ShmHeader* GetHeader() const;
  const char* GetUrdfString() const;
  size_t GetUrdfSize() const;

  const GlobalJointMap* GetGlobalJointMap() const;
  const GlobalJointParams* GetGlobalJointParams() const;
  const RobotTopology* GetRobotTopology() const;
  const UnitConfig* GetUnit(int unit_type) const;
  const drivechain::DriveChainUnitData* GetDriveChainData(int unit_type) const;

  const DynGroupConfigPage* GetDynGroupConfigPage() const;
  const DynGroupConfig* FindDynGroupConfig(int unit_type,
                                           int dyngrp_model) const;
  bool GetDynGroupNumber(int unit_type, int dyngrp_model,
                         int32_t* out_dyngrp_no) const;
  bool GetPowerRegainLimit(int unit_type, int dyngrp_model,
                           double* out_limit) const;

  static const UnitConfig* MapUnitPage(const char* mmap_path,
                                        int unit_type);

 private:
  RobotConfigStore() = default;
  ~RobotConfigStore();

  int fd_ = -1;
  void* mmap_base_ = nullptr;
  size_t mmap_size_ = 0;
  bool owns_memory_ = true;
  const ShmHeader* header_ = nullptr;
  const GlobalJointMap* global_map_ = nullptr;
  const GlobalJointParams* global_params_ = nullptr;
  const RobotTopology* topology_ = nullptr;
};

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_STORE_HPP_
