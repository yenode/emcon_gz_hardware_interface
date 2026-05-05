# EMCON Gazebo Hardware Interface (`emcon_gz_hardware_interface`)

![CI](https://github.com/yenode/emcon_gz_hardware_interface/actions/workflows/build.yml/badge.svg)

A high-performance `ros2_control` SystemInterface that bridges joint states and commands directly with Gazebo Harmonic using native `gz-transport`, bypassing the ROS 2 DDS layer entirely.

## Why this exists (The Problem)

In standard `gz_ros2_control` setups, the controller manager runs inside the Gazebo process and communicates over shared memory. While highly efficient for single-robot setups, this architecture presents significant challenges when:
1. **Network Isolation**: Running complex, custom state estimators or multi-domain networks where simulation traffic (`gz-transport`) must be strictly isolated from your primary `ROS_DOMAIN_ID`.
2. **Multi-Robot Scaling**: Managing fleets of UGV/UAVs where DDS discovery overhead from simulation data congests the network.

## The Solution

`emcon_gz_hardware_interface` acts as a native "data diode" within the `ros2_control` framework. 
It functions as a standard `SystemInterface` in your standalone ROS 2 stack, but uses native `gz::transport` to subscribe to Gazebo joint states and publish joint commands.

### Key Features
* **DDS Bypass**: Zero ROS 2 topics are used for simulation communication.
* **Real-Time Safe**: Implements `realtime_tools::RealtimeBuffer`. The `read()` method uses atomic pointer dereferencing and **never blocks** the control loop, even when Gazebo state callbacks fire at high frequencies.
* **O(1) Joint Lookup**: Optimized hash map lookups ensure low latency regardless of robot complexity.
* **Configurable**: No hardcoded names or topics. Everything is defined in your URDF.

---

## Usage

Simply replace your existing Gazebo hardware interface in your URDF with `emcon_gz_hardware_interface/EmconGzSystemInterface`.

```xml
<ros2_control name="emcon_gz" type="system">
  <hardware>
    <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
    
    <!-- Mandatory Parameters -->
    <param name="bot_name">my_robot</param>
    <param name="world_name">my_world</param>
    
    <!-- Optional: Override the Gazebo joint state topic (Defaults to /world/<world_name>/model/<bot_name>/joint_state) -->
    <!-- <param name="joint_state_topic">/custom/topic</param> -->
  </hardware>

  <joint name="joint1">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Supported Command Interfaces
* `position` (Maps to `/model/<bot_name>/joint/<joint_name>/cmd_pos`)
* `velocity` (Maps to `/model/<bot_name>/joint/<joint_name>/cmd_vel`)
* `effort` (Maps to `/model/<bot_name>/joint/<joint_name>/cmd_force`)

---

## Build Instructions

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/yenode/emcon_gz_hardware_interface.git

# Install dependencies via rosdep
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build
cd ~/ros2_ws
colcon build --packages-select emcon_gz_hardware_interface
```

## Dependencies
* ROS 2 (Tested on Jazzy)
* `ros2_control` & `hardware_interface`
* `realtime_tools`
* Gazebo Harmonic (`gz-transport13`, `gz-msgs10`)
