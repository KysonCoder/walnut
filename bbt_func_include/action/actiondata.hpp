/**
 * @file actiondata.hpp
 * @brief POD data structures for Action/ActionGroup shared memory refactoring
 * @author Claude (refactor)
 * @version 0.2
 * @date 2026-05-09
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2025-10-02  0.1      Yonglin Chi    first edition (original actiondata.hpp)
 * 2026-05-09  0.2      Claude         add ActionData, ActionGroupSharedData
 * 2026-08-08  0.3      Claude         remove dead corner-blend types (ABI break, authorized)
 *
 * New POD types (ActionData, ActionGroupSharedData) are added
 * for shared-memory cross-process usage. Existing legacy types
 * (ACTION_SPEED_DATA, ACTION_ACC_DATA, etc.) remain for backward compatibility.
 */

#ifndef ACTIONDATA_HPP
#define ACTIONDATA_HPP

#include <array>
#include <type_traits>

#include "legacy_robot_types.hpp"
#include "targetgroup.hpp"
#include "pose.hpp"

// Forward declarations
class CTarget;

// ============================================================================
// Motion Type Enum
// ============================================================================
// 2026-09-02: ACTION_MOTION_TYPE 下沉至 func/kinematic 的
// legacy_robot_types.hpp（经 targetgroup.hpp → basetarget.hpp 传递包含）。
// 此处不再定义，仅保留 int 字面量宏供 legacy 代码使用。

#ifndef ACTION_LINEAR
#define ACTION_LINEAR ACTION_LINE
#endif
#ifndef ACTION_CIRCULAR
#define ACTION_CIRCULAR ACTION_CIRCLE
#endif
#ifndef ACTION_SPLINE
#define ACTION_SPLINE 4
#endif
#ifndef ACTION_SEARCH
#define ACTION_SEARCH 6
#endif
#ifndef ACTION_MOTION_TYPE_COUNT
#define ACTION_MOTION_TYPE_COUNT 8
#endif

#define ACTION_INSTANCE_NAME_LEN 32
#define PATH_SAMPLE_TIME 0.004

// ============================================================================
// Legacy Program/Speed Types (formerly provided by confbase/robot.hpp)
// ============================================================================

/**
 * @brief Simple speed descriptor used by CAction::SetSpeed().
 */
typedef struct {
    double tcp_m_s;      ///< TCP linear speed (m/s)
    double joint_rad_s;  ///< Joint speed (rad/s)
} SPEED_DATA;

/**
 * @brief Per-SOC data inside PROGRAM_LINE_DATA.
 */
typedef struct {
    ACTION_MOTION_TYPE motion_type;
    SOC_UNIT_TYPE unit_type;
    bool is_pose_valid;
    POSE pose;
    SOC_JOINTS joints;
    char target_name[ACTION_INSTANCE_NAME_LEN + 1];
    SPEED_DATA speed;
} PROGRAM_LINE_SOC_DATA;

/**
 * @brief Program line descriptor consumed by CActionGroup::SetData().
 */
typedef struct {
    PROGRAM_LINE_SOC_DATA soc_data[MAX_NO_OF_SOC];
} PROGRAM_LINE_DATA;

// ============================================================================
// Speed Profile Definitions
// ============================================================================

/**
 * @brief Speed profile segment types
 */
enum class SpeedProfileSegmentType {
   kAcceleration,   ///< Accelerating segment
   kConstant,       ///< Constant speed segment
   kDeceleration   ///< Decelerating segment
};

/**
 * @brief Speed profile segment for path planning
 */
struct SpeedProfileSegment {
   double start_L{0.0};                     ///< Start L value (0-1)
   double end_L{0.0};                       ///< End L value (0-1)
   double start_speed{0.0};                 ///< Speed at start
   double end_speed{0.0};                   ///< Speed at end
   SpeedProfileSegmentType type;            ///< Segment type
};

/**
 * @brief Path interpolation point for path planning
 */
struct PathInterpolationPoint {
   double L{0.0};                           ///< Path parameter L (0-1)
   double time_s{0.0};                      ///< Time from start
   math::Cartesian3<double> position;       ///< Position at this point
   math::Quaternion<double> orientation;    ///< Orientation at this point
   std::vector<double> joints;              ///< Joint values at this point
   double speed{0.0};                       ///< Speed at this point
   double acc{0.0};                         ///< Acceleration at this point
};

// ============================================================================
// NEW: Shared-memory POD types for cross-process usage
// ============================================================================

#ifndef SHARED_MAX_JOINTS
#define SHARED_MAX_JOINTS 7
#endif

typedef struct
{
   float tcp_m;
   float ori_tcp_m;
   float ori_rad;
   float ext_tcp_m;
   float ext_m;
   float ext_rad;
}
ACTION_ZONE_DATA;

typedef struct
{
   float pos_factor;
   float speed_factor;
   float min_time;
   float max_time;
}
ACTION_STOPPOINT_DATA;

/* union can not be used for endpoint data.
 * stop_data will be read at program stop even if a zone is defined.
 * zone and stop_time are used together. */
typedef struct
{
   ACTION_ZONE_DATA zone;
   ACTION_STOPPOINT_DATA stop_data;
   float stop_time;               /* stop time in coord fine point */
}
ACTION_ENDPOINT_DATA;


enum action_point_type
{
   ACTION_FINE_POINT
};

/* Data for supervision of robot configuration during interpolation. */
/* Use target_calc_master_joint_cnf for the end point when TRUE */
/* else use target_calc_master_joint_no_cnf */
typedef struct
{
   bool joint_supervision;      /* when joint interpolation */
   bool line_supervision;       /* when line interpolation */
   float limit_joint_1;          /* max deviation in wrong direction (radians) */
   float limit_joint_4;
   float limit_joint_6;
}
ACTION_CNF_DATA;

/* Data for lin. acc. path */
typedef struct
{
   bool acc_limit_active;
   float acc_limit;
   bool dec_limit_active;
   float dec_limit;
}
ACTION_TCP_PATH_ACC_DATA;

/*acc data for interpolation*/
typedef struct
{
   double _max_acc;
   double _max_dacc;
   double _max_jerk;
   double _max_djerk;
   double _acc;
   double _dacc;
   double _max_world_acc;
   bool _max_world_acc_is_defined;
   ACTION_TCP_PATH_ACC_DATA _tcp_path_acc_data;
}ACTION_ACC_DATA;

/*
 * type of speed limit
 */
enum ACTION_VLIMIT_TYPES
{
  ACTION_VTCP,
  ACTION_VORI,
  ACTION_VLIN,
  ACTION_VROT,
  ACTION_V_CP,
  ACTION_TIME,
  ACTION_NO_OF_VLIMIT_TYPES
};

enum action_ipol_method
{
   target_line,
   target_joint_123,
   target_joint_456,
   target_joint_156,
   target_joint_134,
   target_joint_567
};

typedef struct
{
   double tcp_m;
   double ori_tcp_m;
   double ori_rad;
   double ext_tcp_m;
   double ext_m;
   double ext_rad;
} STEP_CORNER_DATA;

/* Data for lin. acc. path */
typedef struct
{
   bool acc_limit_active;
   float acc_limit;
   bool dec_limit_active;
   float dec_limit;
} STEP_TCP_PATH_ACC_DATA;


enum action_pos_method
{
   ACTION_POS_NONE,
   ACTION_POS_JOINT_123,
   ACTION_POS_JOINT_134,
   ACTION_POS_LINE,
   ACTION_POS_CIRCLE,
   ACTION_POS_STRAIGHT_CIRCLE
};

enum action_ori_method
{
   ACTION_ORI_NONE,
   ACTION_ORI_JOINT_156,
   ACTION_ORI_JOINT_456,
   ACTION_ORI_JOINT_567,
   ACTION_ORI_QUAT,
   ACTION_ORI_CIRCLE_QUAT,
   ACTION_ORI_CIRCLE_POINT_ORI,
   ACTION_ORI_STRAIGHT_CIRCLE_POINT_ORI,
   ACTION_ORI_ORI_KEEP
};

   typedef struct
{
   double joint_length;   // joint distance for all joints (sqrt of joint distance squared)
   double joint_speed_norm;
   SocVector joint_motion_distance;
   SocVector joint_speed;
   double line_length;
   double tcp_m_s;
   double line_speed_norm;
   double orientation_length;
   double joint_rad_s;
   double orientation_speed_norm;
} ACTION_SPEED_DATA;

typedef struct
{
   ACTION_SPEED_DATA speed[MAX_NO_OF_SOC];
}ACTION_GROUP_SPEED_DATA;

   typedef struct
{
    double step_length_norm;
    double speed_norm;
    double acc_norm;
    double acc_length_norm;
    double dec_length_norm;
    double joint_length;   // joint distance for all joints (sqrt of joint distance squared)

    SocVector joint_motion_distance;  //joint distance for each joint.
    SocVector joint_speed;
    SocVector joint_speed_norm;
    SocVector joint_acc;
    SocVector joint_acc_norm;
    SocVector joint_dec;
    SocVector joint_dec_norm;
    SocVector joint_acc_length_norm;
    SocVector joint_dec_length_norm;
} ACTION_INTERPOLATION_DATA;

// ============================================================================
// Missing type definitions (originally in trajectorygroup.hpp)
// ============================================================================

using JointVector = SOC_JOINTS;

struct JOINTS_GROUP {
    SOC_JOINTS joints[MAX_NO_OF_SOC];
};

// ============================================================================
// NEW: Shared-memory POD types (added 2026-05-09)
// ============================================================================

using ActionGroupVector = std::array<double, MAX_NO_OF_SOC>;

   typedef struct
{
    double step_length_norm;
    double speed_norm;
    double acc_norm;
    double acc_length_norm;
    double dec_length_norm;

    ActionGroupVector soc_joint_motion_length;
    ActionGroupVector soc_step_length_norm;
    ActionGroupVector soc_speed_norm;
    ActionGroupVector soc_acc_norm;
    ActionGroupVector soc_acc_length_norm;
    ActionGroupVector soc_dec_length_norm;

} ACTION_GROUP_INTERPOLATION_DATA;

/**
 * @brief All CActionBase fields as POD for shared memory.
 */
struct ActionData {
    // Identity
    char name[ACTION_INSTANCE_NAME_LEN + 1];
    ACTION_MOTION_TYPE motion_type;
    SOC_UNIT_TYPE unit_type;

    // Interpolation methods
    action_pos_method pos_method;
    action_ori_method ori_method;

    // Targets (embedded, self-contained)
    bool valid_start_target;
    TargetData start_target;
    TargetData end_target;

    // Speed / acceleration / interpolation
    ACTION_SPEED_DATA action_speed;
    ACTION_ACC_DATA action_acc;
    ACTION_INTERPOLATION_DATA action_interpolation;

    // Path parameters
    double Lpos1;
    double Lpos2;
    double predict_action_time;
    double max_deviation_from_path;
    double speed_norm;
    double acc_norm;
    double dacc_norm;
    double jerk_norm;
    double djerk_norm;
    double start_length_norm;
    double end_length_norm;
    double action_length;
    double joint_distance;
    double joint_length;
    double dL;
    double dLmax;
    double length_m;

    // Quaternion redirection
    double fi;
    math::Cartesian3<double> n;
    double fi_tool;

    // End-point and config supervision
    action_point_type end_type;
    ACTION_CNF_DATA cnf;
    ACTION_ENDPOINT_DATA end_data;

    // Frames / load
    bool robot_fixed_object;
    FRAME tool_frame;
    FRAME object_frame;
    double tool_length;
    bool load_defined;

};

/**
 * @brief Shared data for CActionGroup (cross-process shared).
 */
struct ActionGroupSharedData {
    // Per-SOC arrays
    ActionData actions[MAX_NO_OF_SOC];
    bool soc_enable[MAX_NO_OF_SOC]{};
    double soc_master_path_length[MAX_NO_OF_SOC]{};

    // Group-level speed data
    ACTION_GROUP_SPEED_DATA action_group_speed_data;
    ACTION_GROUP_SPEED_DATA action_group_speed;
    ACTION_GROUP_SPEED_DATA pre_action_group_speed;
    ACTION_GROUP_SPEED_DATA next_action_group_speed;

    // Group-level interpolation data
    ACTION_GROUP_INTERPOLATION_DATA action_group_interp_data;

    // Scalar state
    bool is_fine_pose{false};
    int action_group_index{0};
};

// ============================================================================
// POD validation static assertions
// ============================================================================

// Note: std::is_pod requires both standard-layout AND trivial.
// math::Quaternion (and thus math::Pose, FRAME, TargetData) has user-defined
// constructors, so it is not trivial. However, it IS standard-layout and
// trivially-copyable, which is sufficient for shared memory usage.
// We assert standard-layout + trivially-copyable for all new POD types.

static_assert(std::is_standard_layout<ActionData>::value,
              "ActionData must be standard-layout for shared memory");
static_assert(std::is_trivially_copyable<ActionData>::value,
              "ActionData must be trivially-copyable for shared memory");

static_assert(std::is_standard_layout<ActionGroupSharedData>::value,
              "ActionGroupSharedData must be standard-layout for shared memory");
static_assert(std::is_trivially_copyable<ActionGroupSharedData>::value,
              "ActionGroupSharedData must be trivially-copyable for shared memory");

#endif // ACTIONDATA_HPP
