#ifndef FUNC_KINEMATIC_INCLUDE_KINEMATIC_ERROR_INFO_HPP_
#define FUNC_KINEMATIC_INCLUDE_KINEMATIC_ERROR_INFO_HPP_

#include <cstdint>

/// 运动学错误详情（可选输出通道）。POD，shm/实时兼容。
/// 调用方传 nullptr 时零开销（求解器判空即跳）。
/// 2026-10-10：设计报告 §3.2（docs/2026-10-10-kinematic-error-handling-design-report.md）。
struct KinematicErrorInfo {
  int32_t code = 0;             // 主返回码回显（与函数返回值一致）
  int32_t joint_index = -1;     // 涉事关节（-1 = 不适用/多关节）
  int32_t sub_cause = 0;        // 模型原始子码（求解器内部码；0 = 无/与主码相同）
  uint32_t axis_limit_mask = 0; // 超限轴位域（bit i = 轴 i+1）
};

#endif  // FUNC_KINEMATIC_INCLUDE_KINEMATIC_ERROR_INFO_HPP_
