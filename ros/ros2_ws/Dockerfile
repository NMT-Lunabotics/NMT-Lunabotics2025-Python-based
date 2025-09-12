FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    nano \
    git \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

WORKDIR /ros2_ws
RUN mkdir -p src

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
