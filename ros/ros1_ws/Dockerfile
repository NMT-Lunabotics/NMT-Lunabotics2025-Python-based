FROM ubuntu:20.04

ARG USER=luna
ENV DEBIAN_FRONTEND=noninteractive

# Create user
RUN useradd -m $USER && usermod -aG dialout,video,sudo $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV HOME_DIR=/home/$USER
ENV WORKING_DIR=$HOME_DIR/ros1_ws
ENV GIT_REPO=https://github.com/hku-mars/FAST-LIVO2.git
ENV GIT_BRANCH=main

SHELL ["/bin/bash", "-c"]

USER root

# Install base dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release apt-transport-https git build-essential cmake sudo tzdata \
    libopencv-dev libeigen3-dev libpcl-dev libceres-dev \
    libgoogle-glog-dev libgflags-dev libyaml-cpp-dev \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update

# Install ROS Noetic and Python tools
RUN apt-get install -y \
    ros-noetic-ros-base \
    python3-catkin-tools \
    python3-rosdep \
    python3-argcomplete \
    ros-noetic-usb-cam \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-rosconsole \
    rviz \
    && rm -rf /var/lib/apt/lists/*

# Source ROS in bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# Create workspace
RUN mkdir -p $WORKING_DIR/src && chown -R $USER:$USER $WORKING_DIR

USER $USER
WORKDIR $WORKING_DIR/src

# Clone FAST-LIVO2 (public)
RUN git clone --depth 1 -b $GIT_BRANCH $GIT_REPO

WORKDIR $WORKING_DIR

# Initialize rosdep and build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y && catkin_make"

# Add utility scripts (if you have them)
COPY start_docker.sh /usr/local/bin/start_docker
RUN chmod +x /usr/local/bin/start_docker && echo "alias mystart='/usr/local/bin/start_docker'" >> /etc/bash.bashrc
COPY build.sh /usr/local/bin/build_docker
RUN chmod +x /usr/local/bin/build_docker && echo "alias mybuild='/usr/local/bin/build_docker'" >> /etc/bash.bashrc

# Entrypoint
USER root
WORKDIR $HOME_DIR
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

USER $USER
WORKDIR $HOME_DIR

