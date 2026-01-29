# NMT-Lunabotics2025-Python-based
The main repository for the New Mexico Tech Lunabotics 2026 competition Team

<br>
<br>
<br>

# INSTALLATION AND SETUP
Follow the (local) install steps on your respective platform if you wish to run a local version of the robotic system for testing.<br>
Follow the (ssh) steps if you are manually running the robot or testing on the jetson.<br>
Follow the (GUI) steps if you are automatically running the robot using the gui.

***
### (shh) Jetson
1. Join the same network as the target jetson. The Jetson will mostly be connected to our router `team-14`, or it will be connected to `NMT-Weblogin`.
2. ssh into the jetson, using it's name and ip address. The ip address can change, but the most recent ip and usernames are listed below. The jetsons also have a sticker that have the ip address. 
```
ssh luna@192.168.0.207
```

**Jetson infomation:**

| Computer | Username | Ip address | Password |
|-----|-----|-----|-----|
| Main jetson used in eletronics box | luna | 192.168.0.207 | 123456789 |
| Jetson used in goliath test robot | nuc | 192.168.0.139 | 123456789 |
| Other jetson | luna | This jetson has No wifi | 123456789 |


3. Enter the lunabotics repository folder.
```
cd NMT-Lunabotics2025-Python-based/
```
4. Launch the ros system.
```
./start_docker.sh -r -b -d
```

***
<br>


### (Local) Windows 11<br>
(NOTE, windows linux will Not have access to cameras or usb ports)

**Linux and WSL installation:**

1. Install [Ubuntu 22.04.5 LTS](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=US) from Microsoft store.<br>
2. Open (Ubuntu 22.04.5 LTS), let it install and eneter a username and password for the linux system.<br>
3. WSL should be installed by default if it's not, open (command prompt) and install it:<br>
```
wsl --install
```
4. Follow (Local) System setup steps.

***
<br>

### (Local) Linux<br>
1. Follow (Local) System setup steps. 

***
<br>

### (Local) System setup<br>
**Docker installation:**<br>
1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) using the apt repository method inside of Linux terminal.<br>
```
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```
```
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
2. Docker should start on it's own, but to ensure that it's running, start it manually.
```
sudo systemctl start docker
```
3. Give it user access so that it can run the docker containors.
```
sudo usermod -aG docker $USER
```
4. Restart everything so that it can be used. On windows that can be done by simply running the following command in a command prompt and then re-opening (Ubuntu 22.04.5 LTS).
```
wsl --shutdown
```
5. You will know if it worked when the command ``docker ps`` outputs the following in your Linux terminal.
```
CONTAINER ID IMAGE COMMAND CREATED STATUS PORTS NAMES
```

***
<br>

**Repository setup and running:**
1. Inside of your Linux terminal clone the lunabotics repository.
```
git clone https://github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based
```
2. Enter the cloned lunabotics repository folder.
```
cd NMT-Lunabotics2025-Python-based/
```
3. Launch the ros system.
```
./start_docker.sh -r -b -d
```
4. After the system installs and builds you will be successfully running local copy of the system. The terminal will display your current location, something like `root@BenjaminLaptop:/home/luna/NMT-Lunabotics2025-Python-based/ros2_ws#`, and from there you can do whatever. For example the command below will open an rviz2 window.
```
ros2 run rviz2 rviz2
```

***
<br>
<br>
<br>

# USAGE
This script is used to start and manage a Docker and running the whole system. You can start the script by just using `./start_docker.sh`, adding other parameters change how the containor is ran and waht starts up on it's own.
```
Actions (pick ONE):
    --start (-s)                           Start the main system control loop
    --aprial_tag (-tag) [display|d]        Starts the aprial tag position system, and realsense camera IMU sensor system

Options:
    --display (-d)                         Enable display support (forward X11 display)
    --build (-b)                           Build the Docker container (will stop the running container if any)
    --restart (-r)                         Restart all Docker containers
    --mount (-mm)                          Mounts a directory into jetson across wifi
    --pull (-p)                            Pulls the most recent files from github
    --containor (-c)                       Switches between entering the ros or python container with bash
    --help (-h)                            Show this help message
```

***
<br>
<br>
<br>

# TOOLS
1. For local testing without a sensor you can run fake lidar node. This node runs a fake simulation using a fake map, posting fake data that simulates a 2D lidar sensor and aprial tags. The simulator opens a gui where you have full access to the robots position like location, rotation, and camera rotation. However like the real robot to simulate movment by other means you will need to publish commands via the command topic node. 
```
ros2 run fake_lidar fake_lidar_node.py
```
