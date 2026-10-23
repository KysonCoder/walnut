#ifndef BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_BUILDER_HPP_
#define BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_BUILDER_HPP_

#include "shm_layout.hpp"
#include "drivechain/drive_chain_tables.hpp"
#include <cstddef>
#include <memory>

namespace robot_config {

class RobotConfigBuilder {
 public:
  RobotConfigBuilder();
  ~RobotConfigBuilder();

  bool LoadDriveParams(const char* json_config_path);
  bool LoadUrdfKinematics(const char* urdf_path);

  bool SetUnitDriveParams(int unit_type,
                          const JointDriveParams* joints, int n);
  bool SetUnitKinematics(int unit_type,
                         const JointKinematicParams* joints, int n);
  bool SetUnitDynamics(int unit_type,
                       const LinkDynamicParams* links, int n);
  bool SetUnitDriveChain(int unit_type,
                         const drivechain::DriveChainJointConfig* configs,
                         const drivechain::DriveChainJointTables* tables,
                         int n);

  bool SetKinematicModelName(int unit_type, const char* name);
  bool SetUnitRootLink(int unit_type, const char* root_link);
  bool SetUnitTipLink(int unit_type, const char* tip_link);

  bool SetGlobalBaseLink(const char* base_link);
  bool SetKinematicChain(int chain_index, const KinematicChainConfig& chain);
  bool AddDynGroupConfig(const DynGroupConfig& config);
  bool SetDynGroupConfigs(const DynGroupConfig* configs, int n);

  bool Build(const char* output_mmap_path);
  size_t GetTotalSize() const;

  // Returns the minimum buffer size required for BuildToMemory().
  // Returns 0 if no data has been loaded.
  size_t GetSerializedSize() const;

  // Serializes the configured robot data into a caller-provided buffer.
  // out_buffer must be aligned to kShmPageSize and at least GetSerializedSize()
  // bytes large. On success, optionally writes actual bytes written via
  // out_written. The output is byte-identical to Build(path) file contents.
  bool BuildToMemory(void* out_buffer, size_t buffer_size,
                     size_t* out_written = nullptr);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl;

  // Internal serialization without loaded-state checks. Used by Build()
  // (backward compat) and BuildToMemory() (with explicit checks).
  bool SerializeToBuffer(void* out_buffer, size_t buffer_size,
                         size_t* out_written);
};

}  // namespace robot_config

#define ROBOT_CONFIG_BUILDER_HAS_BUILD_TO_MEMORY 1

#endif  // BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_BUILDER_HPP_
