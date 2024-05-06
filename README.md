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

## necessary packages

### ros2 humble

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### colcon

```bash
sudo apt install python3-colcon-common-extensions
```

### gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

### xacro

```bash
sudo apt install ros-humble-xacro
```

### ros2_control

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

### libserial

```bash
sudo apt install libserial-dev
```

### slam_toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

### nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### twistmux

```bash
sudo apt install ros-humble-twist-mux
```

### joystick

```bash
sudo apt install joystick evtest
```

### fields2cover

```bash
sudo apt install ros-humble-fields2cover
 ```

### opennav_coverage

```bash
git clone https://github.com/open-navigation/opennav_coverage.git --branch humble
```

#### change opennav_coverage/CMakeLists.txt

 find_package(Fields2Cover REQUIRED) -> find_package(Fields2Cover REQUIRED HINTS "/opt/ros/humble/cmake")

#### change opennav_row_overage/CMakeLists.txt

 find_package(Fields2Cover REQUIRED) -> find_package(Fields2Cover REQUIRED HINTS "/opt/ros/humble/cmake")

## build Dockerfile

```bash
cd <project folder>
docker build -t <image name> .
```

## run Dockerfile

```bash
docker run -it --name vacuum_cleaner --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --device-cgroup-rule="c *:* rmw" <image name>
```
