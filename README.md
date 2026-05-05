# emcon_gz_hardware_interface

ros2_control SystemInterface that bridges joint commands and states with Gazebo using gz-transport.

## Usage

Add to your URDF:

```xml
<ros2_control name="emcon_gz" type="system">
  <hardware>
    <plugin>emcon_gz_hardware_interface/EmconGzSystemInterface</plugin>
    <param name="bot_name">my_robot</param>
    <param name="world_name">my_world</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Build

```bash
colcon build --packages-select emcon_gz_hardware_interface
```

## Dependencies
- ros2_control
- realtime_tools
- gz-transport13
- gz-msgs10
