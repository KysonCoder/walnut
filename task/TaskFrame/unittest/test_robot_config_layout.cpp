#include "confbase/shm_layout.hpp"
#include <gtest/gtest.h>
#include <type_traits>
#include <cstring>

using namespace robot_config;

TEST(RobotConfigLayout, ShmHeaderFitsInOnePage) {
    EXPECT_LE(sizeof(ShmHeader), kShmPageSize);
}

TEST(RobotConfigLayout, GlobalJointMapIsExactlyOnePage) {
    EXPECT_EQ(sizeof(GlobalJointMap), kShmPageSize);
}

TEST(RobotConfigLayout, GlobalJointParamsIsExactlyOnePage) {
    EXPECT_EQ(sizeof(GlobalJointParams), kShmPageSize);
}

TEST(RobotConfigLayout, UnitConfigFitsInOnePage) {
    EXPECT_LE(sizeof(UnitConfig), kShmPageSize);
}

TEST(RobotConfigLayout, JointDriveParamsSizeIs104) {
    EXPECT_EQ(sizeof(JointDriveParams), 104u);
}

TEST(RobotConfigLayout, JointKinematicParamsSizeIs216) {
    EXPECT_EQ(sizeof(JointKinematicParams), 216u);
}

TEST(RobotConfigLayout, LinkDynamicParamsSizeIs136) {
    EXPECT_EQ(sizeof(LinkDynamicParams), 136u);
}

TEST(RobotConfigLayout, AllStructsAreStandardLayout) {
    static_assert(std::is_standard_layout_v<ShmHeader>);
    static_assert(std::is_standard_layout_v<GlobalJointMap>);
    static_assert(std::is_standard_layout_v<GlobalJointParams>);
    static_assert(std::is_standard_layout_v<UnitConfig>);
    static_assert(std::is_standard_layout_v<JointDriveParams>);
    static_assert(std::is_standard_layout_v<JointKinematicParams>);
    static_assert(std::is_standard_layout_v<LinkDynamicParams>);
    EXPECT_TRUE(true);
}

TEST(RobotConfigLayout, AllStructsAreTriviallyCopyable) {
    static_assert(std::is_trivially_copyable_v<ShmHeader>);
    static_assert(std::is_trivially_copyable_v<GlobalJointMap>);
    static_assert(std::is_trivially_copyable_v<GlobalJointParams>);
    static_assert(std::is_trivially_copyable_v<UnitConfig>);
    EXPECT_TRUE(true);
}

TEST(RobotConfigLayout, UnitConfigHasKinematicFields) {
    robot_config::UnitConfig unit{};
    // These members must exist and be 64-byte char arrays.
    EXPECT_EQ(sizeof(unit.kinematic_model_name), 64u);
    EXPECT_EQ(sizeof(unit.root_link), 64u);
    EXPECT_EQ(sizeof(unit.tip_link), 64u);
    // Simple compile-time check that we can write to them.
    std::strncpy(unit.kinematic_model_name, "CKinGeneRbt", sizeof(unit.kinematic_model_name));
    std::strncpy(unit.root_link, "Body", sizeof(unit.root_link));
    std::strncpy(unit.tip_link, "Rarm_ter", sizeof(unit.tip_link));
}

TEST(RobotConfigLayout, RobotTopologyExists) {
    robot_config::RobotTopology topo{};
    EXPECT_EQ(sizeof(topo.global_base_link), 64u);
    EXPECT_LE(topo.num_chains, static_cast<int32_t>(robot_config::kMaxChains));
}
