# ros2 vacuum cleaner 
ROS2-based robot vacuum cleaner with a differential drive system and LiDAR

## Real robot
- LIFE V4 wheels, sweeper, vaccum -> Pi pico -> Pi -> ros2_control (diff_drive)
- LIFE V4 buttons, distance sensors -> Pi pico -> Pi -> custom node
- Rplidar -> Pi -> rplidar_ros
- Pi -> rviz2, slam

## Simulation
- ros2_control (diff_drive)
- gazebo_ros2_control
- slam
- rviz2

# remap teleop_twist_keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```
