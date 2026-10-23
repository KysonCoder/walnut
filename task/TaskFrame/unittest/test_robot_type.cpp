#include "confbase/robot.hpp"
#include <gtest/gtest.h>

TEST(RobotType, ConstantsMatchSelectedType) {
#if ROBOT_TYPE == HUMANOID
    EXPECT_EQ(robot_config::kRobotMaxJoints, 57u);
    EXPECT_EQ(robot_config::kRobotMaxUnits, 15u);
    EXPECT_EQ(robot_config::kRobotMaxUnitJoints, 7u);
    EXPECT_EQ(robot_config::kRobotMaxUnitLinks, 8u);
#else
    EXPECT_EQ(robot_config::kRobotMaxJoints, 13u);
    EXPECT_EQ(robot_config::kRobotMaxUnits, 3u);
    EXPECT_EQ(robot_config::kRobotMaxUnitJoints, 7u);
    EXPECT_EQ(robot_config::kRobotMaxUnitLinks, 8u);
#endif
}
