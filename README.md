# emcon_hardware_interface

A **ros2_control `SystemInterface`** that bridges joint commands and states between the ROS 2 controller manager and the Gazebo physics engine using **gz-transport directly**, bypassing ROS 2 DDS entirely.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## Why?

The standard `gz_ros2_control` package runs the entire controller manager *inside* the Gazebo process and uses shared memory for I/O. This works well for single-domain setups but introduces problems when:

- You run **custom odometry / state estimation** algorithms and need to isolate simulation DDS traffic from your robot's `ROS_DOMAIN_ID`.
- You operate **multi-robot fleets** where each robot's controller manager must be network-isolated from the physics engine.
- You want the controller manager on a **separate machine** from the Gazebo instance.

EMCON solves this by acting as a lightweight **data diode**: it subscribes to Gazebo joint states and publishes joint commands over `gz::transport`, which uses its own discovery and serialization layer — completely independent of DDS.

## Architecture

```
┌─────────────────────────────┐          gz-transport          ┌──────────────────┐
│  ROS 2 Controller Manager   │  ◄──── joint states ─────────  │                  │
│                              │                                │  Gazebo Harmonic  │
│  EmconSystemInterface        │  ────► joint commands ───────► │  (Physics Engine) │
│  (this package)              │                                │                  │
└─────────────────────────────┘     (no DDS, no ros_gz_bridge)  └──────────────────┘
```

**Key design choices:**
- **Lock-free RT loop** — Uses `realtime_tools::RealtimeBuffer` so the real-time `read()` never blocks, even when Gazebo publishes at high rates.
- **O(1) joint matching** — Uses `std::unordered_map` instead of nested iteration for the gz-transport callback.
- **URDF-driven config** — Bot name and world name are read from standard `<ros2_control><hardware><param>` tags.

## URDF Integration

Add this to your robot's URDF/Xacro:

```xml
<ros2_control name="emcon_system" type="system">
  <hardware>
    <plugin>emcon_hardware_interface/EmconSystemInterface</plugin>
    <param name="bot_name">my_robot</param>
    <param name="world_name">my_world</param>
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
```

## Building

```bash
# Clone into your colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/yenode/emcon_hardware_interface.git

# Build
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select emcon_hardware_interface

# Verify
source install/setup.bash
ros2 pkg list | grep emcon
```

### Dependencies

| Dependency | Purpose |
|---|---|
| `ros2_control` / `hardware_interface` | SystemInterface base class |
| `realtime_tools` | Lock-free RealtimeBuffer for RT safety |
| `gz-transport13` | Direct Gazebo communication |
| `gz-msgs10` | Gazebo message types |
| `pluginlib` | Plugin registration |

## Supported Interfaces

| Command Interface | Gazebo Topic Suffix |
|---|---|
| `position` | `/cmd_pos` |
| `velocity` | `/cmd_vel` |
| `effort` | `/cmd_force` |

State interfaces: `position` and `velocity` are read from Gazebo's `joint_state` Model message.

## License

Apache-2.0 — see [LICENSE](LICENSE).
