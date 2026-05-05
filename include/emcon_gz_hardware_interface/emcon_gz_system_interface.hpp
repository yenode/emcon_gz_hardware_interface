// Copyright 2026 Aditya Pachauri
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCON_GZ_HARDWARE_INTERFACE__EMCON_GZ_SYSTEM_INTERFACE_HPP_
#define EMCON_GZ_HARDWARE_INTERFACE__EMCON_GZ_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "emcon_gz_hardware_interface/visibility_control.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <gz/transport/Node.hh>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/double.pb.h>

namespace emcon_gz_hardware_interface
{

struct JointConfig
{
  std::string name;
  std::string command_interface_type;
  gz::transport::Node::Publisher pub;
};

struct JointStateSnapshot
{
  std::vector<double> positions;
  std::vector<double> velocities;
};

class EmconGzSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EmconGzSystemInterface)

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  EMCON_GZ_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_gz_joint_state(const gz::msgs::Model & msg);

  std::vector<JointConfig> joint_configs_;
  std::vector<double> joint_commands_;
  std::string bot_name_;
  std::string world_name_;
  std::string joint_state_topic_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  realtime_tools::RealtimeBuffer<JointStateSnapshot> state_buffer_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

}

#endif
