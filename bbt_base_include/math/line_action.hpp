/**
 * @file line_action.hpp
 * @brief 直线路径段位姿插值
 * @author Claude
 * @date 2025-04-09
 *
 * 基于归一化长度 l (0 <= l <= 1) 在直线路径段上计算任意一点的位姿
 * 包括3D位置（线性插值）和四元数姿态（球面线性插值Slerp）
 */

#ifndef __LINE_ACTION_HPP__
#define __LINE_ACTION_HPP__

#include "mathpose.h"

/**
 * @brief 在直线路径段上插值计算位姿
 *
 * @param[in] start_pose  起始位姿 (l=0时的位姿)
 * @param[in] end_pose    终止位姿 (l=1时的位姿)
 * @param[in] l           归一化长度参数，范围 [0, 1]
 * @param[out] result     插值结果位姿
 *
 * @note 位置使用线性插值，姿态使用四元数球面线性插值(Slerp)
 * @note l < 0 时返回 start_pose, l > 1 时返回 end_pose
 */
void LineActionInterpolatePose(const POSE* start_pose,
                                const POSE* end_pose,
                                double l,
                                POSE* result);

#endif  // __LINE_ACTION_HPP__
