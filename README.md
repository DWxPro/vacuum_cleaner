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

### opencv

```bash
pip install opencv-python
```

### matplotlib

```bash
pip install matplotlib
```

### Node.js

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
```

-> open new terminal

```bash
nvm install 20
```

### Node-RED

```bash
npm install -g --unsafe-perm node-red
```

on Raspberry Pi you can install Node.js and Node-RED with this script:

```bash
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
```

### BehaviorTree.CPP

install conan

```bash
pip install conan
```

create conan default profil

```bash
conan profile detect
```

clone repository

```bash
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
```

go to parent folder and compile BehaviorTree.CPP

```bash
mkdir build
cd build
conan iconan install ../BehaviorTree.CPP --output-folder=. --build=missing
cmake ../BehaviorTree.CPP -DCMAKE_TOOLCHAIN_FILE="conan_toolchain.cmake"
cmake --build . --parallel
```

### RUN Dockerfile

```bash
docker run -it --name vacuum_cleaner --user ros --network=host --ipc=host -v <path to folder>/vacuum_cleaner/:/home/ros/ros2_ws/src/vacuum_cleaner -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --device-cgroup-rule="c *:* rmw" <image name>
```
