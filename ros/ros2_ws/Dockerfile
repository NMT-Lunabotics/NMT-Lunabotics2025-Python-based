FROM ros:humble-ros-base

# Set up environment
SHELL ["/bin/bash", "-c"]

# Install extra packages if needed (example: colcon, git, build tools)
RUN apt-get update && apt-get install -y \
    #Packages that RVis2 requires.
    ros-humble-rviz-common \
    #Packages that RVis2 for visual display of robot elements.
    ros-humble-rviz-default-plugins \
    #RVis2 package.
    ros-humble-rviz2 \ 
    #
    #
    #        
    nano \              
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Source ROS setup automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
