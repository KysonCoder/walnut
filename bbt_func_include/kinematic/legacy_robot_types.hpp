/**
 * @file legacy_robot_types.hpp
 * @brief Legacy robot type definitions formerly provided by confbase.
 *
 * These types were previously located in confbase/legacy_robot_types.hpp and
 * were removed from confbase during the 2026-06 confbase refactor. They are
 * retained here in func/kinematic because they define the legacy kinematic
 * interface (MAN_ROBOT, SOC_UNIT_TYPE, SOC_JOINTS, etc.) used by CTarget,
 * CArmTarget, and downstream action/pathplanning modules.
 *
 * @author Yonglin Chi / Claude
 * @version 0.2
 * @date 2026-06-28
 */

#ifndef FUNC_KINEMATIC_INCLUDE_LEGACY_ROBOT_TYPES_HPP_
#define FUNC_KINEMATIC_INCLUDE_LEGACY_ROBOT_TYPES_HPP_

#include <array>
#include "pose.hpp"
#include "confbase/robot.hpp"  // ROBOT_TYPE / DUAL_ARM / HUMANOID

#define MAX_ENGINE_NUMBER 6
#define MAX_NO_OF_JOINTS 57      // 机器人驱动轴数
#define ARM_NUMBER 2             // 机器人手臂数量 0：没有手臂, 1：仅有右手臂，2：双手臂
#define ARM_AXIS_NUMBER 7        // 每条手臂的驱动轴数
#define WRIST_AXIS_NUMBER 2      // 每个手腕的轴数
#define THUMB_AXIS_NUMBER 5
#define LITTLE_FINGER_AXIS_NUMBER 3
#define GENERAL_FINGER_AXIS_NUMBER 3
#define HEAD_AXIS_NUMBER 1
#define ROBOT_BODY_AXIS_NUMBER 4
#define AGV_FOOT_AXIS_NUMBER 4
#define WAIST_AXIS_NUMBER 0
#define LEG_NUMBER 0
#define LEG_AXIS_NUMBER 5
#define MAX_NO_OF_SOC 15
#define MAX_NO_OF_OBJECT 10
#define MAX_JOINT_NO_OF_SOC 7

// 机器人配置常量
#define MAN_MAX_NO_OF_JOINTS 6
#define MAN_ROBOT_NO_OF_JOINTS 6
#define MAN_INSTANCE_NAME_LEN 60

typedef struct {
  double joint[MAN_ROBOT_NO_OF_JOINTS];
} MAN_ROBOT_JOINTS;

enum man_error_type {
  man_nominal,
  man_error
};

typedef struct {
  math::Cartesian3<double> joint1;
  math::Cartesian3<double> joint2;
  math::Cartesian3<double> joint3;
  math::Cartesian3<double> joint4;
  math::Cartesian3<double> joint5;
  math::Cartesian3<double> joint6;
  math::Cartesian3<double> joint7;
} MAN_ROBOT_OFFSET_ERROR;

typedef struct {
  double sin_alfa_1, sin_beta_1, gamma_1;
  double cos_alfa_1, cos_beta_1;
  double sin_alfa_2, sin_beta_2, gamma_2;
  double cos_alfa_2, cos_beta_2;
  double sin_alfa_3, sin_beta_3, gamma_3;
  double cos_alfa_3, cos_beta_3;
  double sin_alfa_4, sin_beta_4, gamma_4;
  double cos_alfa_4, cos_beta_4;
  double sin_alfa_5, sin_beta_5, gamma_5;
  double cos_alfa_5, cos_beta_5;
  double sin_alfa_6, sin_beta_6, gamma_6;
  double cos_alfa_6, cos_beta_6;
  double sin_alfa_7, sin_beta_7, gamma_7;
  double cos_alfa_7, cos_beta_7;
} MAN_ROBOT_ATTITUDE_ERROR;

typedef struct {
  enum man_error_type error;
  int no_of_joints;
  bool rotating[MAN_ROBOT_NO_OF_JOINTS];
  math::Cartesian3<double> cp_body_c;
  MAN_ROBOT_OFFSET_ERROR offset;
  MAN_ROBOT_ATTITUDE_ERROR attitude;
  MAN_ROBOT_JOINTS joint_max;
  MAN_ROBOT_JOINTS joint_min;
  MAN_ROBOT_JOINTS joint_compliance;
  float length_of_lower_arm;
  float length_of_upper_arm;
  float length_of_wrist;
  float offset_of_joint2_x;
  float offset_of_upper_arm;
  float height_of_foot;
  float height_of_base;
  float length_of_tube_shaft;
  float length_of_rear_upper_arm;
  float length_of_parallel_bar;
  float length_of_parallel_arm;
  float length_of_housing_axis_4;
  math::Pose<double> tool_interface_pose;
} MAN_ROBOT;

typedef struct {
  math::Pose<double> pose;
  int coord_type;
} FRAME;

enum SOC_UNIT_TYPE {
  NO_UNIT = -1,
  RIGHT_ARM = 0,
  RIGHT_THUMB = 1,
  RIGHT_FF = 2,
  RIGHT_MF = 3,
  RIGHT_RF = 4,
  RIGHT_LF = 5,
  LEFT_ARM = 6,
  LEFT_THUMB = 7,
  LEFT_FF = 8,
  LEFT_MF = 9,
  LEFT_RF = 10,
  LEFT_LF = 11,
  ROBOT_HEAD = 12,
  ROBOT_BODY = 13,
  AGV_FOOT = 14
};

// flavor 语义常量：SOC 槽位的语义值。dualarm 为紧凑布局（0/1/2），
// humanoid 为 legacy 布局（0/6/13）。凡按「这是什么 SOC」判断处
// （路由 switch、左右臂判定）一律用这三个常量，不要直接用 legacy 枚举名。
#if ROBOT_TYPE == DUAL_ARM
constexpr SOC_UNIT_TYPE kSocRightArm = static_cast<SOC_UNIT_TYPE>(0);
constexpr SOC_UNIT_TYPE kSocLeftArm = static_cast<SOC_UNIT_TYPE>(1);
constexpr SOC_UNIT_TYPE kSocBody = static_cast<SOC_UNIT_TYPE>(2);
#else
constexpr SOC_UNIT_TYPE kSocRightArm = RIGHT_ARM;
constexpr SOC_UNIT_TYPE kSocLeftArm = LEFT_ARM;
constexpr SOC_UNIT_TYPE kSocBody = ROBOT_BODY;
#endif

enum TARGET_TYPE {
  NO_TARGET = -1,
  HEAD_TARGET = 0,
  ARM_TARGET = 1,
  THUMB_TARGET = 2,
  GFINGER_TARGET = 3,
  LITTLE_FINGER_TARGET = 4,
  ROBOT_BODY_TARGET = 5,
  AGV_FOOT_TARGET = 6
};

/*
 * Used for the vector of active physical joint numbers (map vector)
 */
typedef struct {
  int map[MAX_NO_OF_JOINTS];
} ROBOT_JOINT_MAP;

typedef math::Cartesian3<double> CARTESIAN;
typedef math::Quaternion<double> QUATERNION;
typedef std::array<double, 7> SOC_JOINTS;
using SocVector = SOC_JOINTS;

typedef struct {
  double joint[MAX_JOINT_NO_OF_SOC];
} JOINT_DATA;

typedef struct {
  CARTESIAN pos;
  QUATERNION orient;
} POSE;

typedef struct {
  SOC_UNIT_TYPE type;
  POSE pose;
  SOC_JOINTS joints;
} SOC_STATUS;

typedef struct {
  SOC_STATUS soc[MAX_NO_OF_SOC];
} ROBOT_STATUS;

// ============================================================================
// Motion Type Enum（2026-09-02 自 func/action/actiondata.hpp 下沉）
// ============================================================================
// ACTION_MOTION_TYPE 最初由 confbase/legacy_robot_types.hpp 提供，confbase
// 重构后暂存于 func/action。因 TargetData（本库 POD）需要携带该字段，
// 枚举回到其历史所在层。底层类型显式固定为 int，保证 shm/ABI 稳定。
// action 侧 actiondata.hpp 改为包含本头文件，源兼容。

enum class ACTION_MOTION_TYPE : int {
    ACTION_NULL = 0,
    ACTION_LINE,
    ACTION_CIRCLE,
    ACTION_JOINT,
    ACTION_SPLINE = 4,
    ACTION_FORCE,
    ACTION_SEARCH = 6,
    ACTION_MOTION_TYPE_COUNT = 8
};

// Global unqualified constants for legacy code that uses bare names.
constexpr ACTION_MOTION_TYPE ACTION_NULL = ACTION_MOTION_TYPE::ACTION_NULL;
constexpr ACTION_MOTION_TYPE ACTION_LINE = ACTION_MOTION_TYPE::ACTION_LINE;
constexpr ACTION_MOTION_TYPE ACTION_CIRCLE = ACTION_MOTION_TYPE::ACTION_CIRCLE;
constexpr ACTION_MOTION_TYPE ACTION_JOINT = ACTION_MOTION_TYPE::ACTION_JOINT;
constexpr ACTION_MOTION_TYPE ACTION_FORCE = ACTION_MOTION_TYPE::ACTION_FORCE;

#endif  // FUNC_KINEMATIC_INCLUDE_LEGACY_ROBOT_TYPES_HPP_
