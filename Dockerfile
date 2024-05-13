FROM ubuntu:jammy
SHELL ["/bin/bash", "-c"]

# < --- UPDATE --- >

RUN apt-get update -y \
 && apt-get upgrade -y \
 && rm -rf /var/lib/apt/lists/*

# < --- SETTINGS --- >

# user settings
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# ros2 workspace
ENV PACKAGE_NAME=vacuum_cleaner
ENV ROS_DISTRO=iron
ARG ros2_ws=/home/$USERNAME/ros2_ws

# create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && mkdir /home/$USERNAME/.config \
 && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# set up sudo
RUN apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*

# use noninteratvie mode for apt 
ARG DEBIAN_FRONTEND=noninteractive

# set timezone 
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# allow access to serial devices
RUN usermod -aG dialout $USERNAME

# add lines to .bashrc
USER $USERNAME
RUN echo -e '\n\n' '### <---- own stuff ----> ### \n' \
            "source /opt/ros/$ROS_DISTRO/setup.bash \n" \
            'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
            'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc
USER root
RUN echo -e '\n\n' '### <---- own stuff ----> ### \n' \
            "source /opt/ros/$ROS_DISTRO/setup.bash \n" \
            'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
            'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc

# create workspace
RUN mkdir -p $ros2_ws

# < --- INSTALLATION --- >

# nano
RUN apt-get update \
 && apt-get install -y nano \
 && rm -rf /var/lib/apt/lists/*
 
 # twist-mux
RUN apt-get update \
 && apt-get install -y git \
 && rm -rf /var/lib/apt/lists/*

# ros2
RUN apt-get update \
 && apt-get install -y software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

RUN apt-get update \
 && apt-get install -y curl \
 && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update \
 && apt install -y ros-$ROS_DISTRO-desktop \
 && rm -rf /var/lib/apt/lists/*

# colcon
RUN apt-get update \
 && apt-get install -y python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# gazebo
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
 && rm -rf /var/lib/apt/lists/*

#joystick
RUN apt-get update \
 && apt-get install -y joystick \
    evtest\
 && rm -rf /var/lib/apt/lists/*

# libserial-dev
RUN apt-get update \
 && apt-get install -y libserial-dev \
 && rm -rf /var/lib/apt/lists/*

# xacro
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-xacro \
 && rm -rf /var/lib/apt/lists/*

# ros2-control
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
 && rm -rf /var/lib/apt/lists/*

# slam
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-slam-toolbox \
 && rm -rf /var/lib/apt/lists/*

# nav2
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
 && rm -rf /var/lib/apt/lists/*

# twist-mux
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-twist-mux \
 && rm -rf /var/lib/apt/lists/*

# opennav_coverage
RUN apt-get update \
 && apt-get install -y ros-$ROS_DISTRO-fields2cover \
 && rm -rf /var/lib/apt/lists/*

WORKDIR $ros2_ws/src
RUN git clone https://github.com/open-navigation/opennav_coverage.git -b $ROS_DISTRO \
 && sed -i "s|find_package(Fields2Cover REQUIRED)|find_package(Fields2Cover REQUIRED HINTS \"/opt/ros/$ROS_DISTRO/cmake\")|g" $ros2_ws/src/opennav_coverage/opennav_coverage/CMakeLists.txt \
 && sed -i "s|find_package(Fields2Cover REQUIRED)|find_package(Fields2Cover REQUIRED HINTS \"/opt/ros/$ROS_DISTRO/cmake\")|g" $ros2_ws/src/opennav_coverage/opennav_row_coverage/CMakeLists.txt

# < --- WORKSPACE --- >

# gezebo world model
RUN mkdir -p /home/$USERNAME/.gazebo/models/Home1
COPY ./worlds/Home1 /home/$USERNAME/.gazebo/models/Home1

# ros2 workspace
RUN mkdir -p $ros2_ws/src/$PACKAGE_NAME
COPY . $ros2_ws/src/$PACKAGE_NAME

# < --- BUILD --- >

WORKDIR $ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.sh \
 && colcon build --symlink-install\
 && ldconfig

# < --- PERMISSIONS --- >
RUN chown -R $USERNAME:$USER_GID /home/$USERNAME