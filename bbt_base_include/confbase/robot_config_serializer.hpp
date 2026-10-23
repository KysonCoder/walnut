#ifndef BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_SERIALIZER_HPP_
#define BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_SERIALIZER_HPP_

#include "robot_config_common.hpp"
#include "humanoid_conf.hpp"
#include "dualarm_conf.hpp"
#include <cstddef>
#include <type_traits>

namespace robot_config {
namespace internal {

class RobotConfigSerializer {
 public:
  RobotConfigSerializer();
  ~RobotConfigSerializer();

  RobotConfigSerializer(const RobotConfigSerializer&) = delete;
  RobotConfigSerializer& operator=(const RobotConfigSerializer&) = delete;

  template <typename Layout>
  bool SerializeToFile(const Layout* layout, size_t size, const char* path);

  template <typename Layout>
  ConfigError Attach(const char* path, const Layout** out_layout);

  template <typename Layout>
  ConfigError AttachToMemory(const void* base, size_t size,
                             const Layout** out_layout);

  void Detach();
  bool IsAttached() const;

  uint32_t ComputeCrc32(const void* data, size_t size) const;

 private:
  int fd_;
  void* mmap_base_;
  size_t mmap_size_;
  bool owns_memory_;

  template <typename Layout>
  ConfigError ValidateLayoutBuffer(const Layout* header, size_t size) const;
};

}  // namespace internal
}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_ROBOT_CONFIG_SERIALIZER_HPP_
