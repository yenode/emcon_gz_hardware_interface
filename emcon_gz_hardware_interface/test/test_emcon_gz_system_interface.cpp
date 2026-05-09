// Copyright 2026 Aditya Pachauri
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lifecycle_helpers.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

const char kValidUrdf[] =
  R"(
<?xml version="1.0"?>
<robot name="test_bot">
  <ros2_control name="emcon_gz_test" type="system">
    <hardware>
      <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
      <param name="bot_name">test_robot</param>
      <param name="world_name">test_world</param>
    </hardware>
    <joint name="wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <link name="base_link"/>
  <link name="wheel_link"/>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)";

const char kMissingBotNameUrdf[] =
  R"(
<?xml version="1.0"?>
<robot name="test_bot">
  <ros2_control name="emcon_gz_missing_bot" type="system">
    <hardware>
      <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
      <param name="world_name">test_world</param>
    </hardware>
    <joint name="wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <link name="base_link"/>
  <link name="wheel_link"/>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)";

const char kMissingWorldNameUrdf[] =
  R"(
<?xml version="1.0"?>
<robot name="test_bot">
  <ros2_control name="emcon_gz_missing_world" type="system">
    <hardware>
      <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
      <param name="bot_name">test_robot</param>
    </hardware>
    <joint name="wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <link name="base_link"/>
  <link name="wheel_link"/>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)";

const char kCustomTopicUrdf[] =
  R"(
<?xml version="1.0"?>
<robot name="test_bot">
  <ros2_control name="emcon_gz_custom_topic" type="system">
    <hardware>
      <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
      <param name="bot_name">my_bot</param>
      <param name="world_name">my_world</param>
      <param name="joint_state_topic">/custom/joint_states</param>
    </hardware>
    <joint name="steer_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <link name="base_link"/>
  <link name="steer_link"/>
  <joint name="steer_joint" type="revolute">
    <parent link="base_link"/>
    <child link="steer_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
</robot>
)";

const char kMultiJointUrdf[] =
  R"(
<?xml version="1.0"?>
<robot name="test_bot">
  <ros2_control name="emcon_gz_multi" type="system">
    <hardware>
      <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
      <param name="bot_name">multi_bot</param>
      <param name="world_name">sim_world</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)";

std::shared_ptr<hardware_interface::ResourceManager> make_resource_manager(
  const std::string & urdf)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto logger = rclcpp::get_logger("test_emcon_gz");
  return std::make_shared<hardware_interface::ResourceManager>(urdf, clock, logger);
}

// Returns true if the named component failed to initialize (is in UNKNOWN/ERROR state).
bool component_failed_to_init(
  const std::shared_ptr<hardware_interface::ResourceManager> & rm,
  const std::string & component_name)
{
  const auto status_map = rm->get_components_status();
  auto it = status_map.find(component_name);
  if (it == status_map.end()) {
    return true;
  }
  const auto & state = it->second.state;
  return state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN ||
         state.label() == hardware_interface::lifecycle_state_names::UNKNOWN;
}

}  // namespace

class EmconGzSystemInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Verify the plugin loads and registers the correct state and command interfaces.
TEST_F(EmconGzSystemInterfaceTest, LoadsAndExposesInterfaces)
{
  auto rm = make_resource_manager(kValidUrdf);

  auto state_interfaces = rm->state_interface_keys();
  EXPECT_EQ(state_interfaces.size(), 2u);
  EXPECT_NE(
    std::find(state_interfaces.begin(), state_interfaces.end(), "wheel_joint/position"),
    state_interfaces.end());
  EXPECT_NE(
    std::find(state_interfaces.begin(), state_interfaces.end(), "wheel_joint/velocity"),
    state_interfaces.end());

  auto command_interfaces = rm->command_interface_keys();
  EXPECT_EQ(command_interfaces.size(), 1u);
  EXPECT_NE(
    std::find(command_interfaces.begin(), command_interfaces.end(), "wheel_joint/velocity"),
    command_interfaces.end());
}

// Missing bot_name: the component must be left in an error/unconfigured state.
TEST_F(EmconGzSystemInterfaceTest, FailsWithMissingBotName)
{
  auto rm = make_resource_manager(kMissingBotNameUrdf);
  EXPECT_TRUE(component_failed_to_init(rm, "emcon_gz_missing_bot"));
}

// Missing world_name: the component must be left in an error/unconfigured state.
TEST_F(EmconGzSystemInterfaceTest, FailsWithMissingWorldName)
{
  auto rm = make_resource_manager(kMissingWorldNameUrdf);
  EXPECT_TRUE(component_failed_to_init(rm, "emcon_gz_missing_world"));
}

// Optional joint_state_topic override must be accepted without error.
TEST_F(EmconGzSystemInterfaceTest, AcceptsCustomJointStateTopic)
{
  auto rm = make_resource_manager(kCustomTopicUrdf);
  EXPECT_FALSE(component_failed_to_init(rm, "emcon_gz_custom_topic"));
}

// Two joints must produce four state interfaces and two command interfaces.
TEST_F(EmconGzSystemInterfaceTest, MultiJointInterfaceCount)
{
  auto rm = make_resource_manager(kMultiJointUrdf);
  EXPECT_EQ(rm->state_interface_keys().size(), 4u);
  EXPECT_EQ(rm->command_interface_keys().size(), 2u);
}

// read() and write() must return OK after the hardware is activated.
TEST_F(EmconGzSystemInterfaceTest, ReadWriteReturnOk)
{
  auto rm = make_resource_manager(kValidUrdf);

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
  rm->set_component_state("emcon_gz_test", active_state);

  const rclcpp::Time t(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);

  auto read_result = rm->read(t, dt);
  EXPECT_EQ(read_result.result, hardware_interface::return_type::OK);

  auto write_result = rm->write(t, dt);
  EXPECT_EQ(write_result.result, hardware_interface::return_type::OK);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
