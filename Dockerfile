#FROM osrf/ros:humble-desktop-full
FROM ubuntu:jammy

# update base image
RUN apt-get update -y \
 && apt-get upgrade -y \
 && rm -rf /var/lib/apt/lists/*

# user settings
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# ros2 workspace
ARG ros2_ws=/home/$USERNAME/ros2_ws/

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

# allow access to serial devices
RUN usermod -aG dialout $USERNAME


ARG DEBIAN_FRONTEND=noninteractive

# set timezone 
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


# ros2 workspace
#COPY config/ $ros2_ws/src/config/
#use this after developing. In the meantime use bind mount

# ros2
RUN apt-get update \
 && apt-get install -y software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

RUN apt-get update \
 &&  apt-get install -y curl \
 && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update \
 && apt install -y ros-humble-desktop \
 && rm -rf /var/lib/apt/lists/*

# colcon
RUN apt-get update \
 && apt-get install -y python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# gazebo
RUN apt-get update \
 && apt-get install -y ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
 && rm -rf /var/lib/apt/lists/*

# nano
RUN apt-get update \
 && apt-get install -y nano \
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
 && apt-get install -y ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

# ros2-control
RUN apt-get update \
 && apt-get install -y ros-humble-ros2-control \
    ros-humble-ros2-controllers \
 && rm -rf /var/lib/apt/lists/*

# slam
RUN apt-get update \
 && apt-get install -y ros-humble-slam-toolbox \
 && rm -rf /var/lib/apt/lists/*

# nav2
RUN apt-get update \
 && apt-get install -y ros-humble-navigation2 \
    ros-humble-nav2-bringup \
 && rm -rf /var/lib/apt/lists/*

# twist-mux
RUN apt-get update \
 && apt-get install -y ros-humble-twist-mux \
 && rm -rf /var/lib/apt/lists/*


# add lines to .bashrc
USER ros
RUN echo '\n\n' '### <---- own stuff ----> ### \n' \
         'source /opt/ros/humble/setup.bash \n' \
         'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
         'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc
USER root
RUN echo '\n\n' '### <---- own stuff ----> ### \n' \
         'source /opt/ros/humble/setup.bash \n' \
         'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
         'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc

