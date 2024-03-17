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
## xacro
```bash
sudo apt install ros-humble-xacro
```
## ros2_control 
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
## libserial
```bash
sudo apt install libserial-dev
```
## slam_toolbox
```bash
sudo apt install ros-humble-slam-toolbox
```
## nav2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
## twistmux
```bash
sudo apt install ros-humble-twist-mux
```
## joystick
```bash
sudo apt install joystick evtest
```
## joystick
```bash
sudo apt install joystick evtest
```
## fields2cover
```bash
sudo apt-get update
sudo apt-get install --no-install-recommends software-properties-common
sudo apt-get install --no-install-recommends build-essential ca-certificates cmake \
    doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
    python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev
python3 -m pip install gcovr

git clone https://github.com/Fields2Cover/Fields2Cover.git

cd Fields2Cover
mkdir -p build
cd build;
cmake -DBUILD_PYTHON=ON ..
make -j$(nproc)
sudo make install

 ```
## opencv
```bash
pip install opencv-python
```
## matplotlib
```bash
pip install matplotlib
```
