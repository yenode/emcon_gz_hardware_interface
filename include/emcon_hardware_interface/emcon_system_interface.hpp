// Copyright 2026 Silent Sentry Project
// SPDX-License-Identifier: Apache-2.0
//
// EMCON — ros2_control SystemInterface
// Bridges joint commands/states to Gazebo via gz-transport,
// bypassing ROS 2 DDS entirely.

#ifndef EMCON_HARDWARE_INTERFACE__EMCON_SYSTEM_INTERFACE_HPP_
#define EMCON_HARDWARE_INTERFACE__EMCON_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "emcon_hardware_interface/visibility_control.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// gz-transport — must be fully included (no forward decl due to inline templates)
#include <gz/transport/Node.hh>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/double.pb.h>

namespace emcon_hardware_interface
{

/// Static per-joint configuration. Set during on_init, immutable at runtime.
struct JointConfig
{
  std::string name;

  /// "position", "velocity", "effort", or "" (read-only joint)
  std::string command_interface_type;

  /// gz-transport publisher for this joint's command topic
  gz::transport::Node::Publisher pub;
};

/// Snapshot of all joint states received from Gazebo.
/// Exchanged lock-free between gz-transport I/O thread and the RT control loop
/// via RealtimeBuffer.
struct JointStateSnapshot
{
  std::vector<double> positions;
  std::vector<double> velocities;
};

class EmconSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EmconSystemInterface)

  // ── lifecycle callbacks ──

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── realtime loop ──

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  EMCON_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// gz-transport callback — runs on gz-transport's I/O thread (non-RT)
  void on_gz_joint_state(const gz::msgs::Model & msg);

  // ── data ──

  /// Per-joint static configuration (immutable after on_init)
  std::vector<JointConfig> joint_configs_;

  /// Latest command values per joint (owned by the RT thread)
  std::vector<double> joint_commands_;

  /// Gazebo model name (set from URDF hardware param "bot_name")
  std::string bot_name_;

  /// Gazebo world name (set from URDF hardware param "world_name")
  std::string world_name_;

  /// gz-transport node (heap-allocated to defer construction past on_init)
  std::unique_ptr<gz::transport::Node> gz_node_;

  /// Lock-free state exchange: gz-transport callback writes, RT read() reads.
  /// Replaces std::mutex to guarantee the RT loop never blocks.
  realtime_tools::RealtimeBuffer<JointStateSnapshot> state_buffer_;

  /// O(1) joint name → index lookup for the gz-transport callback
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

}  // namespace emcon_hardware_interface

#endif  // EMCON_HARDWARE_INTERFACE__EMCON_SYSTEM_INTERFACE_HPP_
