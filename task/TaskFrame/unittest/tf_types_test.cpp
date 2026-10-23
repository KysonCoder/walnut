#include <gtest/gtest.h>
#include <type_traits>
#include "tf_types.hpp"

TEST(TfTypes, EventMappingRoundTrip) {
    EXPECT_EQ(EventTypeToLegacy(EventType::EMPTY), EMPTY_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::INIT), INIT_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::CONFIG), CONFIG_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::CONTROL_ON), CONTROL_ON_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::CONTROL_OFF), CONTROL_OFF_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::MOTOR_ON), MOTOR_ON_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::MOTOR_OFF), MOTOR_OFF_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::ERROR), ERROR_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::PATH_STOP), PATH_STOP_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::EMERGENCY_STOP), EMERGENCY_STOP_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::SAFE_STOP), SAFE_STOP_EVENT);
    EXPECT_EQ(EventTypeToLegacy(EventType::RESET), RESET_EVENT);
    for (int i = 0; i <= 11; ++i) {
        EXPECT_EQ(LegacyToEventType(EventTypeToLegacy(static_cast<EventType>(i))),
                  static_cast<EventType>(i));
    }
}

TEST(TfTypes, LegacyValuesUnchanged) {
    // 下游二进制兼容：位掩码数值不可变
    EXPECT_EQ(EMPTY_EVENT, 0);
    EXPECT_EQ(INIT_EVENT, 1 << 0);
    EXPECT_EQ(RESET_EVENT, 1 << 10);
    EXPECT_EQ(MAX_TASK_NUMBER, 32);
}

TEST(TfTypes, FeedbackSizesFromConfbase) {
#if ROBOT_TYPE == HUMANOID
    EXPECT_EQ(robot_config::kRobotMaxJoints, 57u);
    EXPECT_EQ(robot_config::kRobotMaxUnits, 15u);
#elif ROBOT_TYPE == DUAL_ARM
    EXPECT_EQ(robot_config::kRobotMaxJoints, 13u);
    EXPECT_EQ(robot_config::kRobotMaxUnits, 3u);
#endif
    FEEDBACK_ALL_JOINTS j;
    EXPECT_EQ(sizeof(j.joints) / sizeof(j.joints[0]), robot_config::kRobotMaxJoints);
    FEEDBACK_ALL_POSE p;
    EXPECT_EQ(sizeof(p.soc_pose) / sizeof(p.soc_pose[0]), robot_config::kRobotMaxUnits);
    FEEDBACK_ALL_FORCE f;
    EXPECT_EQ(sizeof(f.soc_force) / sizeof(f.soc_force[0]), robot_config::kRobotMaxUnits);
}

TEST(TfTypes, ShmStructConstraints) {
    static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_JOINTS>);
    static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_POSE>);
    static_assert(std::is_trivially_copyable_v<FEEDBACK_ALL_FORCE>);
    static_assert(std::is_trivially_copyable_v<MESSAGE_DATA>);
    SUCCEED();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
