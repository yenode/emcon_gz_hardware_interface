// Copyright 2026 Silent Sentry Project
// SPDX-License-Identifier: Apache-2.0
//
// EMCON — ros2_control SystemInterface implementation.
// Uses gz-transport to communicate joint commands/states directly with
// the Gazebo physics engine, bypassing ROS 2 DDS entirely.

#include "emcon_hardware_interface/emcon_system_interface.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace emcon_hardware_interface
{

// ═══════════════════════════════════════════════════════════════════════════
//  on_init — Parse joint configuration from URDF HardwareInfo
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn EmconSystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Call the parent on_init (stores hardware_info into info_)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  // Read custom hardware parameters from URDF <ros2_control><hardware><param>
  auto it_bot = info_.hardware_parameters.find("bot_name");
  bot_name_ = (it_bot != info_.hardware_parameters.end()) ?
    it_bot->second : "autobot";

  auto it_world = info_.hardware_parameters.find("world_name");
  world_name_ = (it_world != info_.hardware_parameters.end()) ?
    it_world->second : "thar_desert";

  RCLCPP_INFO(
    get_logger(),
    "[EMCON] Initializing for model='%s' in world='%s'",
    bot_name_.c_str(), world_name_.c_str());

  // ── Parse joints from the URDF <ros2_control> block ──
  const size_t n_joints = info_.joints.size();
  joint_configs_.resize(n_joints);
  joint_commands_.resize(n_joints, 0.0);

  // Seed the RealtimeBuffer with a zero-initialised snapshot so that
  // readFromRT() always returns valid data, even before Gazebo publishes.
  JointStateSnapshot initial_snapshot;
  initial_snapshot.positions.resize(n_joints, 0.0);
  initial_snapshot.velocities.resize(n_joints, 0.0);
  state_buffer_.writeFromNonRT(initial_snapshot);

  for (size_t i = 0; i < n_joints; ++i) {
    joint_configs_[i].name = info_.joints[i].name;

    // Determine command type from the URDF command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      joint_configs_[i].command_interface_type =
        info_.joints[i].command_interfaces[0].name;
    } else {
      // Read-only joint (e.g. front wheel encoders)
      joint_configs_[i].command_interface_type = "";
    }

    // Build reverse lookup map for O(1) matching in the gz-transport callback
    joint_name_to_index_[joint_configs_[i].name] = i;

    RCLCPP_INFO(
      get_logger(),
      "[EMCON]   Joint[%zu]: '%s'  cmd_type='%s'",
      i, joint_configs_[i].name.c_str(),
      joint_configs_[i].command_interface_type.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_configure — Stand up the gz-transport node and subscribe to joint states
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn EmconSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[EMCON] Creating gz-transport node...");
  gz_node_ = std::make_unique<gz::transport::Node>();

  // Subscribe to the Gazebo joint state topic for this model
  // Gazebo publishes Model messages at: /world/<world>/model/<model>/joint_state
  const std::string joint_state_topic =
    "/world/" + world_name_ + "/model/" + bot_name_ + "/joint_state";

  if (!gz_node_->Subscribe(
        joint_state_topic, &EmconSystemInterface::on_gz_joint_state, this))
  {
    RCLCPP_ERROR(
      get_logger(),
      "[EMCON] Failed to subscribe to gz-transport topic: %s",
      joint_state_topic.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "[EMCON] Subscribed to gz-transport: %s", joint_state_topic.c_str());

  // ── Advertise joint command topics ──
  for (auto & jc : joint_configs_) {
    if (jc.command_interface_type.empty()) {
      continue;
    }

    std::string cmd_suffix;
    if (jc.command_interface_type == hardware_interface::HW_IF_POSITION) {
      cmd_suffix = "cmd_pos";
    } else if (jc.command_interface_type == hardware_interface::HW_IF_VELOCITY) {
      cmd_suffix = "cmd_vel";
    } else if (jc.command_interface_type == hardware_interface::HW_IF_EFFORT) {
      cmd_suffix = "cmd_force";
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[EMCON] Unsupported command interface '%s' for joint '%s', skipping.",
        jc.command_interface_type.c_str(), jc.name.c_str());
      continue;
    }

    const std::string topic =
      "/model/" + bot_name_ + "/joint/" + jc.name + "/" + cmd_suffix;

    jc.pub = gz_node_->Advertise<gz::msgs::Double>(topic);
    RCLCPP_INFO(
      get_logger(), "[EMCON] Advertised command topic: %s", topic.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_activate — Ready for real-time loop
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn EmconSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[EMCON] Activating — data diode is live.");

  // Seed commands from the latest Gazebo state so there's no jerk on activation
  const auto * snapshot = state_buffer_.readFromNonRT();
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & cmd_type = joint_configs_[i].command_interface_type;
    if (cmd_type == hardware_interface::HW_IF_POSITION) {
      joint_commands_[i] = snapshot->positions[i];
    } else if (cmd_type == hardware_interface::HW_IF_VELOCITY ||
      cmd_type == hardware_interface::HW_IF_EFFORT)
    {
      joint_commands_[i] = 0.0;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_deactivate
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn EmconSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[EMCON] Deactivating — stopping commands.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_cleanup — Tear down gz-transport
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn EmconSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[EMCON] Cleaning up gz-transport node.");
  gz_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  read — Copy latest gz-transport state into ros2_control state interfaces
//
//  RT-SAFE: readFromRT() is an atomic pointer dereference — never blocks.
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type EmconSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto * snapshot = state_buffer_.readFromRT();

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & name = joint_configs_[i].name;

    const std::string pos_key = name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_key = name + "/" + hardware_interface::HW_IF_VELOCITY;

    if (has_state(pos_key)) {
      set_state(pos_key, snapshot->positions[i]);
    }
    if (has_state(vel_key)) {
      set_state(vel_key, snapshot->velocities[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  write — Beam commands to Gazebo via gz-transport
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type EmconSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    auto & jc = joint_configs_[i];

    // Skip read-only joints (no command interface)
    if (jc.command_interface_type.empty()) {
      continue;
    }

    // Read the latest command from the framework
    const std::string cmd_key = jc.name + "/" + jc.command_interface_type;
    if (has_command(cmd_key)) {
      joint_commands_[i] = get_command<double>(cmd_key);
    }

    if (!jc.pub.Valid()) {
      continue;
    }

    if (!std::isnan(joint_commands_[i])) {
      gz::msgs::Double msg;
      msg.set_data(joint_commands_[i]);

      // Non-blocking publish over gz-transport
      jc.pub.Publish(msg);
    }
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  gz-transport callback — Runs on gz-transport's I/O thread (non-RT)
//
//  Builds a complete snapshot and swaps it into the RealtimeBuffer.
//  The RT thread (read()) picks it up via an atomic pointer dereference.
// ═══════════════════════════════════════════════════════════════════════════
void EmconSystemInterface::on_gz_joint_state(const gz::msgs::Model & msg)
{
  const size_t n = joint_configs_.size();

  // Start from the current snapshot so unmentioned joints keep their values
  JointStateSnapshot snapshot;
  snapshot.positions.resize(n, 0.0);
  snapshot.velocities.resize(n, 0.0);

  const auto * current = state_buffer_.readFromNonRT();
  if (current != nullptr) {
    snapshot = *current;
  }

  // O(1) lookup per Gazebo joint via unordered_map
  for (int j = 0; j < msg.joint_size(); ++j) {
    const auto & gz_joint = msg.joint(j);
    auto it = joint_name_to_index_.find(gz_joint.name());
    if (it == joint_name_to_index_.end()) {
      continue;  // Joint not in our URDF config
    }

    const size_t idx = it->second;
    if (gz_joint.has_axis1()) {
      snapshot.positions[idx] = gz_joint.axis1().position();
      snapshot.velocities[idx] = gz_joint.axis1().velocity();
    }
  }

  state_buffer_.writeFromNonRT(snapshot);
}

}  // namespace emcon_hardware_interface

// ── pluginlib registration ──
PLUGINLIB_EXPORT_CLASS(
  emcon_hardware_interface::EmconSystemInterface,
  hardware_interface::SystemInterface)
