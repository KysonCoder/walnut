#ifndef TASK_SERVO_SERVO_STEP_DATA_HPP_
#define TASK_SERVO_SERVO_STEP_DATA_HPP_

#include "confbase/robot.hpp"

namespace servo {

struct ServoStepMotorData {
    bool is_valid;
    double inertia;
    double compl_inertia;
    double dist_torque;
    double gravity_torque;
    double abs_max_friction;
    double abs_min_friction;
    bool col_detect_on;
    double col_detect_level;
    bool ref_filter_coeff_valid;
    double pos_m;
    double pos_m_physical;
    double delta_pos_m;
    double speed_m;
    double acc_m;
    double acc_arm_m;
    double speed_ffw_m;
    double torque_ffw_m;
    double torque_dist_m;
    double brake_torque_m;
    double torque_ffw_gravity;
    double Kp;
    double Kv;
    double Ki;
    double torque_limit;
    double emergency_stop_dec;
    double emergency_stop_dist_torque;
    bool mechanical_brake_active;
    bool force_control_active;
    double torque_ref_m;
    int joint_no;
};

// 布局护栏：task 层 shm 环形队列直接依赖该布局，不得改变
static_assert(sizeof(ServoStepMotorData) == 240,
              "ServoStepMotorData layout must stay stable (shm ring queue)");

struct ServoStepData {
    ServoStepMotorData motor_step[robot_config::kRobotMaxJoints];
};

#if ROBOT_TYPE == HUMANOID
static_assert(robot_config::kRobotMaxJoints == 57,
              "ServoStepData layout is pinned to 57 joints (HUMANOID)");
static_assert(sizeof(ServoStepData) == 240 * 57,
              "ServoStepData layout must stay stable (shm ring queue)");
#elif ROBOT_TYPE == DUAL_ARM
static_assert(robot_config::kRobotMaxJoints == 13,
              "ServoStepData layout is pinned to 13 joints (DUAL_ARM)");
static_assert(sizeof(ServoStepData) == 240 * 13,
              "ServoStepData layout must stay stable (shm ring queue)");
#endif

}  // namespace servo

#endif
