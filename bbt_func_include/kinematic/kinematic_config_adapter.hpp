/**
 * @file kinematic_config_adapter.hpp
 * @brief Adapter to read kinematic configuration from confbase shared memory.
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2026-06-25
 *
 * Provides real-time-safe accessors to robot configuration data stored in
 * confbase shared memory. The adapter does not own the memory; it delegates
 * to RobotConfigStore (typed alias via robot.hpp) via Attach/Detach semantics.
 */

#ifndef FUNC_KINEMATIC_INCLUDE_KINEMATIC_CONFIG_ADAPTER_HPP_
#define FUNC_KINEMATIC_INCLUDE_KINEMATIC_CONFIG_ADAPTER_HPP_

#include <cstddef>
#include <cstdint>

#include "legacy_robot_types.hpp"
#include "confbase/robot.hpp"
#include "confbase/shm_layout.hpp"

namespace kinematic {

#if ROBOT_TYPE == DUAL_ARM
/**
 * @brief Map a target-group slot index to DualArmSocUnitType.
 *
 * In DUAL_ARM builds the CTargetGroup slot layout is the compact dual-arm
 * layout (RIGHT_ARM=0, LEFT_ARM=1, ROBOT_BODY=2), identical to the typed
 * enum. Legacy SOC_UNIT_TYPE values (e.g. LEFT_ARM==6) no longer map.
 */
inline robot_config::DualArmSocUnitType ToTypedSocUnitType(SOC_UNIT_TYPE unit) {
  switch (static_cast<int>(unit)) {
    case 0:
      return robot_config::DualArmSocUnitType::RIGHT_ARM;
    case 1:
      return robot_config::DualArmSocUnitType::LEFT_ARM;
    case 2:
      return robot_config::DualArmSocUnitType::ROBOT_BODY;
    default:
      return robot_config::DualArmSocUnitType::NO_UNIT;
  }
}
#else
/**
 * @brief Map legacy SOC_UNIT_TYPE to HumanoidSocUnitType.
 *
 * The legacy SOC_UNIT_TYPE enum matches the Humanoid layout values, so this is
 * an identity mapping.
 */
inline robot_config::HumanoidSocUnitType ToTypedSocUnitType(SOC_UNIT_TYPE unit) {
  return static_cast<robot_config::HumanoidSocUnitType>(unit);
}
#endif

#if ROBOT_TYPE == DUAL_ARM
/**
 * @brief Map typed DualArmSocUnitType to the CTargetGroup slot index.
 *
 * DUAL_ARM builds use the compact slot layout: the slot index equals the
 * typed enum value (RIGHT_ARM=0, LEFT_ARM=1, ROBOT_BODY=2).
 */
inline SOC_UNIT_TYPE ToLegacySocUnitType(robot_config::DualArmSocUnitType unit) {
  switch (unit) {
    case robot_config::DualArmSocUnitType::RIGHT_ARM:
      return static_cast<SOC_UNIT_TYPE>(0);
    case robot_config::DualArmSocUnitType::LEFT_ARM:
      return static_cast<SOC_UNIT_TYPE>(1);
    case robot_config::DualArmSocUnitType::ROBOT_BODY:
      return static_cast<SOC_UNIT_TYPE>(2);
    default:
      return NO_UNIT;
  }
}
#else
/**
 * @brief Map typed HumanoidSocUnitType back to legacy SOC_UNIT_TYPE.
 *
 * The legacy SOC_UNIT_TYPE enum matches the Humanoid layout values, so this is
 * an identity mapping.
 */
inline SOC_UNIT_TYPE ToLegacySocUnitType(robot_config::HumanoidSocUnitType unit) {
  return static_cast<SOC_UNIT_TYPE>(unit);
}
#endif

#if ROBOT_TYPE == DUAL_ARM
/** @brief Preferred name: typed unit -> CTargetGroup slot index. */
inline SOC_UNIT_TYPE ToSlotIndex(robot_config::DualArmSocUnitType unit) {
  return ToLegacySocUnitType(unit);
}
#else
/** @brief Preferred name: typed unit -> CTargetGroup slot index. */
inline SOC_UNIT_TYPE ToSlotIndex(robot_config::HumanoidSocUnitType unit) {
  return ToLegacySocUnitType(unit);
}
#endif

/**
 * @brief Adapter for reading kinematic configuration from confbase.
 *
 * Real-time safe: no heap allocation, no exceptions, no virtual calls in
 * accessors. All methods return bool and write into caller-provided buffers.
 */
class KinematicConfigAdapter {
 public:
  KinematicConfigAdapter();

  /**
   * @brief Attach to a RobotConfigStore instance.
   *
   * RobotConfigStore is a typed alias (DualArmConfigStore or HumanoidConfigStore
   * depending on ROBOT_TYPE), so this single overload covers both robot forms.
   * @param store Reference to the store to attach to.
   * @return true if attached successfully.
   */
  bool Attach(robot_config::RobotConfigStore& store);

  /** @brief Detach from the store. */
  void Detach();

  /** @brief Whether the adapter is attached to a store. */
  bool IsAttached() const;

  /**
   * @brief Get the UnitConfig for a given SOC unit type.
   * @param unit SOC_UNIT_TYPE value.
   * @param out_unit Output pointer to the UnitConfig.
   * @return true if found and out_unit was set.
   */
  bool GetUnitConfig(SOC_UNIT_TYPE unit,
                     const robot_config::UnitConfig** out_unit) const;

  /**
   * @brief Get the kinematic model name for a unit.
   * @param unit SOC_UNIT_TYPE value.
   * @param out_name Output pointer to the model name string.
   * @return true if found and out_name was set.
   */
  bool GetKinematicModelName(SOC_UNIT_TYPE unit,
                             const char** out_name) const;

  /**
   * @brief Copy kinematic parameters for a unit into caller-provided buffer.
   * @param unit SOC_UNIT_TYPE value.
   * @param out_params Caller-provided buffer of JointKinematicParams.
   * @param max_count Maximum number of joints to copy (size of out_params).
   * @param out_copied Output: number of joints actually copied.
   * @return true if parameters were copied successfully.
   */
  bool CopyKinematicParams(
      SOC_UNIT_TYPE unit,
      robot_config::JointKinematicParams* out_params,
      int32_t max_count,
      int32_t* out_copied) const;

  /**
   * @brief Get the URDF string from the config store.
   * @param out_urdf Output pointer to the URDF string.
   * @param out_size Output: size of the URDF string.
   * @return true if URDF string is available.
   */
  bool GetUrdfString(const char** out_urdf, size_t* out_size) const;

  /**
   * @brief Get the robot topology from the config store.
   * @param out_topology Output pointer to the RobotTopology.
   * @return true if topology is available.
   */
  bool GetRobotTopology(const robot_config::RobotTopology** out_topology) const;

  /**
   * @brief Return the default kinematic model name for a given unit type.
   * @param unit SOC_UNIT_TYPE value.
   * @return Statically-known model name, or empty string for unknown units.
   */
  static const char* DefaultKinematicModelName(SOC_UNIT_TYPE unit);

 private:
  enum class StoreType {
    kNone,
    kDualArm,
    kHumanoid
  };

  StoreType store_type_ = StoreType::kNone;
  void* store_ = nullptr;
};

}  // namespace kinematic

#endif  // FUNC_KINEMATIC_INCLUDE_KINEMATIC_CONFIG_ADAPTER_HPP_
