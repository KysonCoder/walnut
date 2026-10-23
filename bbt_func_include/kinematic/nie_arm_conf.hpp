/**
 * @file nie_arm_conf.hpp
 * @brief NieNieDual 13-DOF 双臂机器人常量配置
 *
 * 三部分架构:
 *   第1部分: 1-DOF 腰部 (Z轴旋转, L0=156mm 抬升)
 *   第2部分: 6-DOF 左臂 (MDH: Y-X-X-Y-X-Y 拟人臂+球腕)
 *   第3部分: 6-DOF 右臂 (与左臂对称, 基座绕Z多转180°)
 *
 * @date 2026-06-16
 */

#ifndef NIE_ARM_CONF_HPP_
#define NIE_ARM_CONF_HPP_

#include <cmath>

// ── 13-DOF 关节索引 ─────────────────────────────────────
#define NIE_TOTAL_JOINTS   13       // 总关节数: 腰1 + 左臂6 + 右臂6
#define NIE_WAIST_INDEX    0        // 腰关节
#define NIE_LEFT_START      1        // 左臂起始索引
#define NIE_RIGHT_START     7        // 右臂起始索引
#define NIE_PER_ARM         6        // 单臂关节数

// ── 几何参数 (m) ────────────────────────────────────────
// 2026-10-09：毫米→米制化。confbase 注入路径（URDF 派生 MDH）本就是米制，
// fallback 缺省统一为米制，消除「空参回退=毫米 / conf 注入=米」混单位隐患。
#define NIE_L0              0.156    // base→waist Z轴抬升
#define NIE_L1              0.100    // 大臂基座偏移 d1
#define NIE_L2              0.068    // 小臂长度 a3
#define NIE_L3              0.1185   // 大臂长度 d4
#define NIE_LT              0.091    // 工具长度

// ── MDH参数 (单臂) ──────────────────────────────────────
// alpha[i]  a[i]  d[i]  theta_offset[i]  (i=0..5)
constexpr double NIE_ALPHA[] = {
    -M_PI / 2.0,  M_PI / 2.0,  0.0,
    -M_PI / 2.0,  M_PI / 2.0, -M_PI / 2.0
};
constexpr double NIE_A[] =  {0.0, 0.0, NIE_L2, 0.0, 0.0, 0.0};
constexpr double NIE_D[] =  {NIE_L1, 0.0, 0.0, NIE_L3, 0.0, 0.0};
constexpr double NIE_THETA_OFFSET[] = {
     M_PI / 2.0,  M_PI / 2.0, -M_PI / 2.0,
     0.0,         0.0,         0.0
};

#endif  // NIE_ARM_CONF_HPP_
