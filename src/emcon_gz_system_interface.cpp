// Copyright 2026 Aditya Pachauri
// SPDX-License-Identifier: Apache-2.0

#include "emcon_gz_hardware_interface/emcon_gz_system_interface.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace emcon_gz_hardware_interface
{

hardware_interface::CallbackReturn EmconGzSystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  auto it_bot = info_.hardware_parameters.find("bot_name");
  bot_name_ = (it_bot != info_.hardware_parameters.end()) ?
    it_bot->second : "autobot";

  auto it_world = info_.hardware_parameters.find("world_name");
  world_name_ = (it_world != info_.hardware_parameters.end()) ?
    it_world->second : "thar_desert";

  RCLCPP_INFO(
    get_logger(),
    "[EmconGz] Initializing model='%s' in world='%s'",
    bot_name_.c_str(), world_name_.c_str());

  const size_t n_joints = info_.joints.size();
  joint_configs_.resize(n_joints);
  joint_commands_.resize(n_joints, 0.0);

  JointStateSnapshot initial_snapshot;
  initial_snapshot.positions.resize(n_joints, 0.0);
  initial_snapshot.velocities.resize(n_joints, 0.0);
  state_buffer_.writeFromNonRT(initial_snapshot);

  for (size_t i = 0; i < n_joints; ++i) {
    joint_configs_[i].name = info_.joints[i].name;

    if (!info_.joints[i].command_interfaces.empty()) {
      joint_configs_[i].command_interface_type =
        info_.joints[i].command_interfaces[0].name;
    } else {
      joint_configs_[i].command_interface_type = "";
    }

    joint_name_to_index_[joint_configs_[i].name] = i;

    RCLCPP_INFO(
      get_logger(),
      "[EmconGz]   Joint[%zu]: '%s'  cmd_type='%s'",
      i, joint_configs_[i].name.c_str(),
      joint_configs_[i].command_interface_type.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmconGzSystemInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  gz_node_ = std::make_unique<gz::transport::Node>();

  const std::string joint_state_topic =
    "/world/" + world_name_ + "/model/" + bot_name_ + "/joint_state";

  if (!gz_node_->Subscribe(
        joint_state_topic, &EmconGzSystemInterface::on_gz_joint_state, this))
  {
    RCLCPP_ERROR(
      get_logger(),
      "[EmconGz] Failed to subscribe to topic: %s",
      joint_state_topic.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "[EmconGz] Subscribed to topic: %s", joint_state_topic.c_str());

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
      continue;
    }

    const std::string topic =
      "/model/" + bot_name_ + "/joint/" + jc.name + "/" + cmd_suffix;

    jc.pub = gz_node_->Advertise<gz::msgs::Double>(topic);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmconGzSystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "[EmconGz] Activating.");

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

hardware_interface::CallbackReturn EmconGzSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "[EmconGz] Deactivating.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmconGzSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  gz_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EmconGzSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
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

hardware_interface::return_type EmconGzSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    auto & jc = joint_configs_[i];

    if (jc.command_interface_type.empty()) {
      continue;
    }

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
      jc.pub.Publish(msg);
    }
  }

  return hardware_interface::return_type::OK;
}

void EmconGzSystemInterface::on_gz_joint_state(const gz::msgs::Model & msg)
{
  const size_t n = joint_configs_.size();
  JointStateSnapshot snapshot;
  snapshot.positions.resize(n, 0.0);
  snapshot.velocities.resize(n, 0.0);

  const auto * current = state_buffer_.readFromNonRT();
  if (current != nullptr) {
    snapshot = *current;
  }

  for (int j = 0; j < msg.joint_size(); ++j) {
    const auto & gz_joint = msg.joint(j);
    auto it = joint_name_to_index_.find(gz_joint.name());
    if (it == joint_name_to_index_.end()) {
      continue;
    }

    const size_t idx = it->second;
    if (gz_joint.has_axis1()) {
      snapshot.positions[idx] = gz_joint.axis1().position();
      snapshot.velocities[idx] = gz_joint.axis1().velocity();
    }
  }

  state_buffer_.writeFromNonRT(snapshot);
}

}

PLUGINLIB_EXPORT_CLASS(
  emcon_gz_hardware_interface::EmconGzSystemInterface,
  hardware_interface::SystemInterface)
