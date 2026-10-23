#ifndef BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_BUILDER_HPP_
#define BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_BUILDER_HPP_

#include "dualarm_conf.hpp"
#include <cstddef>
#include <memory>

namespace robot_config {

class DualArmConfigBuilder {
 public:
  DualArmConfigBuilder();
  ~DualArmConfigBuilder();

  bool Init();

  bool LoadDriveParams(const char* json_config_path);
  bool LoadUrdfKinematics(const char* urdf_path);

  bool Build(const char* output_mmap_path);
  size_t GetTotalSize() const;
  size_t GetSerializedSize() const;

  bool BuildToMemory(void* out_buffer, size_t buffer_size,
                     size_t* out_written = nullptr);

 private:
  size_t ComputeTotalSize() const;
  bool SerializeToBuffer(void* out_buffer, size_t buffer_size,
                         size_t* out_written);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_DUALARM_CONFIG_BUILDER_HPP_
