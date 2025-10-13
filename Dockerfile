# Set base docker image
FROM python:3.11-slim

# Create non-root user (luna) with access to full containor
ARG USER=luna
RUN useradd -m $USER && usermod -aG dialout,video $USER \
    && apt-get update && apt-get install -y sudo git nano openssh-client rsync \
    && usermod -aG sudo $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Set default Docker settings
ENV HOME_DIR=/home/$USER
ENV WORKING_DIR=$HOME_DIR/NMT-Lunabotics2025-Python-based/system_operations
ENV HOST_FILES_DIR=/host_files  

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Set USER back to root for package installation.
USER root 

# System Packages to install
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    #Essiental packages .ie fundelmental compatition package that are required to run the robot
    #Development packages:
    nano \
    git \
    x11-apps \
    curl \
    rsync \
    && rm -rf /var/lib/apt/lists/*

# Python Packages to install
RUN pip install --no-cache-dir \
    #Essiental packages .ie fundelmental compatition package that are required to run the robot
    pyserial 

# Install used github distros(cli libray)
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
USER luna
ENV PATH="/root/bin:${PATH}"
RUN mkdir -p /home/luna/.arduino15 && chmod -R 777 /home/luna/.arduino15
RUN arduino-cli config init --config-file /home/luna/.arduino15/arduino-cli.yaml \
    && arduino-cli core update-index \
    && arduino-cli core install arduino:avr


# Copy start_docker.sh into docker and make executable for accessing the containor
USER root
COPY start_docker.sh /usr/local/bin/start_docker
RUN chmod +x /usr/local/bin/start_docker \
    && echo "alias mystart='/usr/local/bin/start_docker'" >> /etc/bash.bashrc

# Create working directory for project
RUN mkdir -p $WORKING_DIR \
    && chown -R $USER:$USER $HOME_DIR

# Copy entrypoint.sh into containor, make executable, and run to start the default robot behavoirs
COPY entry_point.sh /entry_point.sh
RUN chmod +x /entry_point.sh
ENTRYPOINT ["/entry_point.sh"]

# Switch back to luna and set default working directory
USER $USER
WORKDIR $HOME_DIR