// func/dyn_humanoid/include/dyn_humanoid_types.hpp
#ifndef FUNC_DYN_HUMANOID_INCLUDE_DYN_HUMANOID_TYPES_HPP_
#define FUNC_DYN_HUMANOID_INCLUDE_DYN_HUMANOID_TYPES_HPP_

#include <array>
#include <cstdint>

#include "math/se3_twist_ops.hpp"
#include "math/pose.hpp"

namespace dyn_humanoid {

// ---- DOF 规模（spec §3）----
constexpr int kWholeBodyDof = 19;
constexpr int kHandDof = 17;
constexpr int kNumFingers = 5;
constexpr int kMaxPhalanx = 5;
constexpr int kMaxSupportPoints = 8;

// ---- 固定数组类型（对外接口只用这些，pinocchio/Eigen 不出现在头文件）----
using VecW = std::array<double, kWholeBodyDof>;
using VecH = std::array<double, kHandDof>;
using Vec2 = std::array<double, 2>;
using Vec3 = se3_twist::Vec3;
using Vec6 = se3_twist::Vec6;
using MatW = std::array<std::array<double, kWholeBodyDof>, kWholeBodyDof>;
using Mat6W = std::array<std::array<double, kWholeBodyDof>, 6>;
// 单指 Jacobian：6x5（拇指 5 列为上限；3-DOF 指仅前 3 列有效）
using Mat6xFinger = std::array<std::array<double, kMaxPhalanx>, 6>;
// 单链 Jacobian：6x7（最大链 DOF = 手臂 7；腰/头链仅前列有效）
using Mat6xChain = std::array<std::array<double, 7>, 6>;
using Pose = math::Pose<double>;

// ---- 全身 19-DOF 布局：WJ1-4 | head_J | LAJ1-7 | RAJ1-7 ----
enum class Chain : int32_t { kWaist = 0, kHead = 1, kLeftArm = 2, kRightArm = 3 };
constexpr int kNumChains = 4;
constexpr int kChainOffset[kNumChains] = {0, 4, 5, 12};
constexpr int kChainDof[kNumChains] = {4, 1, 7, 7};

// ---- 单手 17-DOF 布局：thumb x5, FF/MF/RF/LF x3（指内基到梢）----
constexpr int kFingerDof[kNumFingers] = {5, 3, 3, 3, 3};
constexpr int kFingerOffset[kNumFingers] = {0, 5, 8, 11, 14};

enum class HandSide : int32_t { kLeft = 0, kRight = 1 };

// 五指手占位几何（SI 单位）。Init 可整体覆盖；未来从 confbase 手指单元读取。
struct DexHandGeometry {
  double finger_base_pos[kNumFingers][3];  // 指根在 palm 系位置 (m)
  double finger_base_rpy[kNumFingers][3];  // 指根在 palm 系朝向 (rad)
  int32_t finger_dof[kNumFingers];         // 必须等于 kFingerDof
  double phalanx_len[kNumFingers][kMaxPhalanx];  // 每指节长度 (m)，超出 dof 的槽为 0
  double phalanx_radius;                   // 均匀圆柱半径 (m)
  double density;                          // 占位密度 (kg/m^3)
  double palm_size[3];                     // 平板盒尺寸 (m)
  double palm_mass;                        // 手掌质量 (kg)
};

// 内置占位参数（常人手比例）。仅供框架验证，非真机参数。
DexHandGeometry DefaultDexHandGeometry();

}  // namespace dyn_humanoid

#endif  // FUNC_DYN_HUMANOID_INCLUDE_DYN_HUMANOID_TYPES_HPP_
