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
ARG PACKAGE_NAME=vacuum_cleaner

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
            'source /opt/ros/iron/setup.bash \n' \
            'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
            'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc
USER root
RUN echo -e '\n\n' '### <---- own stuff ----> ### \n' \
            'source /opt/ros/iron/setup.bash \n' \
            'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n' \
            'source /home/ros/ros2_ws/install/setup.bash \n' \
        >> ~/.bashrc

# < --- INSTALLATION --- >

# nano
RUN apt-get update \
 && apt-get install -y nano \
 && rm -rf /var/lib/apt/lists/*
 
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
 && apt install -y ros-iron-desktop \
 && rm -rf /var/lib/apt/lists/*

# colcon
RUN apt-get update \
 && apt-get install -y python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# gazebo
RUN apt-get update \
 && apt-get install -y ros-iron-gazebo-ros-pkgs \
    ros-iron-gazebo-ros2-control \
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
 && apt-get install -y ros-iron-xacro \
 && rm -rf /var/lib/apt/lists/*

# ros2-control
RUN apt-get update \
 && apt-get install -y ros-iron-ros2-control \
    ros-iron-ros2-controllers \
 && rm -rf /var/lib/apt/lists/*

# slam
RUN apt-get update \
 && apt-get install -y ros-iron-slam-toolbox \
 && rm -rf /var/lib/apt/lists/*

# nav2
RUN apt-get update \
 && apt-get install -y ros-iron-navigation2 \
    ros-iron-nav2-bringup \
 && rm -rf /var/lib/apt/lists/*

# twist-mux
RUN apt-get update \
 && apt-get install -y ros-iron-twist-mux \
 && rm -rf /var/lib/apt/lists/*

# Fields2Cover
RUN apt-get update \
 && apt-get install -y --no-install-recommends software-properties-common \
 && apt-get install -y --no-install-recommends build-essential ca-certificates cmake \
    doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
    python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev \
 && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install gcovr

WORKDIR /home/$USERNAME/
RUN git clone https://github.com/Fields2Cover/Fields2Cover.git

WORKDIR /home/$USERNAME/Fields2Cover
RUN mkdir -p build
WORKDIR /home/$USERNAME/Fields2Cover/build
RUN cmake -DBUILD_PYTHON=ON .. \
 && make -j$(nproc) \
 && make install

# < --- WORKSPACE --- >

USER $USERNAME 

# gezebo world model
RUN mkdir -p /home/$USERNAME/.gazebo/models/Home1
COPY ./worlds/Home1 /home/$USERNAME/.gazebo/models/Home1

# ros2 workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src/$PACKAGE_NAME
COPY . /home/$USERNAME/ros2_ws/src/$PACKAGE_NAME

USER root

# < --- BUILD --- >

WORKDIR /home/$USERNAME/ros2_ws
RUN source /opt/ros/iron/setup.sh && colcon build

USER $USERNAME
