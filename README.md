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

# necessary packages
xacro
```bash
sudo apt install ros-humble-xacro
```
ros2_control 
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
libserial
```bash
sudo apt install libserial-dev
```
slam_toolbox
```bash
sudo apt install ros-humble-slam-toolbox
```
nav2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
twistmux
```bash
sudo apt install ros-humble-twist-mux
```
joystick
```bash
sudo apt install joystick evtest
```