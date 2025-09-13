FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    python3-opencv \
    python3-colcon-common-extensions \
    ros-humble-image-tools \
    python3-rosdep \
    #nano--> file editing
    nano \
    ros-$ROS_DISTRO-camera-ros \
    #git--> push.pull from github repsoitory
    git \
    #openssh-client--> ssh connection
    openssh-client \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN mkdir -p src

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
