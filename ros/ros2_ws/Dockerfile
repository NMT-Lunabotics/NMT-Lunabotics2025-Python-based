#Select the ros distribution we are using.
FROM ros:humble

#Create a user to avoid using root, make sure luna doesn't require passwords.
ARG USER=luna
RUN useradd -m $USER && usermod -aG dialout,video $USER
RUN apt-get update && apt-get install -y sudo \
    && usermod -aG sudo $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


#---VARIABLE SETTINGS---
ENV HOME_DIR=/home/$USER
ENV WORKING_DIR=$HOME_DIR/ros2_ws
ENV GIT_REPO=git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git
ENV GIT_BRANCH=benjaminstestbranch
ENV GIT_USER=benjamin-p15
ENV GIT_EMAIL=benjamin.peterson@student.nmt.edu


#Set the locale to use "bash" as default shell.
SHELL ["/bin/bash", "-c"]

#Install our images used packages, and switch to root for process.
USER root
RUN apt-get update && apt-get install -y \
    #camera librarys
    ros-humble-camera-ros \
    ros-humble-usb-cam \
    ros-humble-image-tools \
    ros-humble-image-view \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    v4l-utils \
    #tools
    git \
    openssh-client \
    nano \
#ros-humble-v4l2-camera \
#ros-humble-image-tools \
#ros-$ROS_DISTRO-libcamera-ros \
#ros-$ROS_DISTRO-camera-ros \
#ros-$ROS_DISTRO-demos \
#v4l-utils \
&& rm -rf /var/lib/apt/lists/*


#Copy in ros workspace.
#COPY --chown=$USER:$USER ros/ros2_ws $WORKING_DIR

# Copy script → make executable → add alias
COPY start_docker.sh /usr/local/bin/start_docker
RUN chmod +x /usr/local/bin/start_docker && echo "alias mystart='/usr/local/bin/start_docker'" >> /etc/bash.bashrc
COPY build.sh /usr/local/bin/build_docker
RUN chmod +x /usr/local/bin/build_docker && echo "alias mystart='/usr/local/bin/build_docker'" >> /etc/bash.bashrc

#Set correct user and directory.
USER $USER
ENV ROS_USER=$USER
WORKDIR $WORKING_DIR

#Setup our entry point script.
USER root
WORKDIR $HOME_DIR
#RUN chmod -R 666 /dev read access for everyone
COPY entrypoint.sh /entrypoint.sh
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f $WORKING_DIR/install/setup.bash ]; then source $WORKING_DIR/install/setup.bash; fi" >> /etc/bash.bashrc
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

#Switch to our non root user.
USER $USER
WORKDIR $HOME_DIR
