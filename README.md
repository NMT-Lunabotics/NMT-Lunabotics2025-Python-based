# NMT-Lunabotics2025-Python-based
The main repository for the New Mexico Tech Lunabotics 2026 competition Team

### Table of Contents:

**Installation:**
- The [(coding)](#coding-visual-studio-code-and-github-setup) setup guide explains how to setup Visual studio code and github for writting code and pushing/pulling code from the repository.<br>
- The [(local)](#local-windows-11) setup guide explains how to install and run a local version of the ros system for testing on your respective system.<br>

**Running system** 
- The [(shh)](#shh-jetson) guide walks through sshing onto the jetsons for running and testing the system.<br>
- The [(gui)](#gui-system-gui) walk you through the steps needed to run the gui which can run the ros system withput the need of terminal commands.<br> 

**Docker** <br>
- The [(usage)](#usage) section lists the docker commands that our custom bash file can execute for running docker and ros.<br>
- The [(tools)](#tools) section lists usful tools that can help out when working with the system.<br>

<br>
<br>
<br>

# INSTALLATION AND SETUP

### (shh) Jetson
1. Join the same network as the target jetson. The Jetson will mostly likely be connected to our router `team-14`, or it will be connected to `NMT-Weblogin`.
2. ssh into the jetson, using it's name and ip address. The ip address can change, but the most recent ip and usernames are listed below. The jetsons also have a sticker that lists the ip address. 
```
ssh -X luna@192.168.0.207
```

**Jetson infomation:**

| Computer | Username | Ip address | Password |
|-----|-----|-----|-----|
| Main jetson used in eletronics box | luna | 192.168.0.207 | 123456789 |
| --> Ethernet | luna | 192.168.10.2 | 123456789 |
| Jetson used in goliath test robot | nuc | 129.138.171.148 | 123456789 |
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

### (gui) system gui
TODO: update section

### (Local) Windows 11<br>
(NOTE, windows linux will Not have access to cameras or usb ports)

**Linux and WSL installation:**

1. Install [Ubuntu 22.04.5 LTS](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=US) from Microsoft store.<br>
2. Open (Ubuntu 22.04.5 LTS), let it install and eneter a username and password for the linux system.<br>
3. WSL should be installed by default if it's not, open (command prompt) and install it:<br>
```
wsl --install
```
4. Follow [(local system setup)](#local-system-setup) steps to install docker and run ros.

***
<br>

### (Local) Linux<br>
1. Follow [(local system setup)](#local-system-setup) steps to install docker and run ros.<br>
TODO: expand section
***
<br>

### (Local) System setup<br>
**Docker installation:**<br>
1. Install Docker using the [apt repository method](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) inside of Linux terminal.<br>
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

**Running system:**
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

### (coding) Visual studio code and Github setup:
1. Download and install [Visual studio code](https://code.visualstudio.com/download) with the default settings.

2. Once installed in visual studio code select the (Source Control) tab. If git is not installed Visual studio code will promt you to install it. Install it using the default options. Restart visual studio code after it is installed.

3. Install [Github pull requests](https://marketplace.visualstudio.com/items?itemName=GitHub.vscode-pull-request-github) from the exstensions tab. estart visual studio code after it is installed.

4. After a few seconds under the buttom left profile section the (signin with github) option will be visible. Sign into your github account using that option.

5. Config your git email and password by opening a new terminal, and setting your username and email.
```
git config --global user.email "email@gmail.com"
git config --global user.name "username"
``` 

6. Clone the repository by going to the source control tab, and clicking (clone repository). The top bar will ask for the repostirtoy url: `https://github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based`

7. Github and Visual studio code is now setup. You can pull most recent changes by running: `git pull` in a new terminal. And you can push changes by entering the source control tab, adding a commit message, and clicking (commit and sync).

***
<br>
<br>
<br>

# USAGE
```
Usage: ./start_docker.sh [--display | --build | --restart | --mount | --pull | --arduino | --containor | --quiet | --stop | --help] [--start | --aprial_tag | --usb-cam | --command]
This script is used to start and manage Docker and run the whole system. You can start the script by just using ./start_docker.sh, adding other parameters that change how the container is run and what starts up on its own.

Actions (pick ONE):
  --start (-s)                           Start the main system control loop
  --command (-cmd) <command>             Execute a command inside the container without entering the container
  --aprial_tag (-tag) [display|d]        Starts the aprial tag position system, and realsense camera IMU sensor
  --usb-cam (-u)                         Launch system cameras
Options:
  --display (-d)                         Enable display support (forward X11 display)
  --build (-b)                           Build the Docker container (will stop the running container if any)
  --restart (-r)                         Restart all Docker containers
  --mount (-m) <username> <host_path>    Mounts a directory into jetson across wifi
  --pull (-p)                            Pulls the most recent files from github
  --arduino (-sys)                       Force updates arduino without requiring a full build
  --container (-c) [ros|python]          Switches between entering the ros or python container with bash
  --quiet (-q)                           Suppress bash messages
  --stop (-x)                            Stop the running Docker container
  --help (-h)                            Show this help message
```

***
<br>
<br>
<br>

# TOOLS
**ros simulator:**<br>
For local testing without a sensor you can run fake lidar node. This node runs a fake simulation using a fake map, posting fake data that simulates a 2D lidar sensor and aprial tags. The simulator opens a gui where you have full access to the robots position like location, rotation, and camera rotation. However like the real robot to simulate movment by other means you will need to publish commands via the command topic node. 
```
ros2 run fake_lidar fake_lidar_node.py
```
***
**uplode code:**<br>
For the quick testing of code two option were made for fast iteration. These methiods do not need to be used when testing locally but are usful while working on testing the main system.
<br>
1. The first is github, it's suggested that you setup [Visual stuido code and github](#coding-visual-studio-code-and-github-setup). Doing this allows you to quickly push code to github, and then on the jetson end you can quickly pull and recompile the code using `./start_docker.sh -r -p -b`. The arduino's code will be also updated everytime the containor rebuilds, the `--arduino` flag can be used to force an update or in the system_control folder there are multiple ways to uploude code to the arduino yourself.

2. The second way is a volume mount which makes the jetson use files from your pc.<br>
TODO: update section

3. Safty gaurd: Since linux systems can nativly execute .sh files, there's a chance that you may run `./start_docker.sh -p` which will pull files from github force overwriting un-saved work. To prevent this local repositorys should include a file named `.tripwire` which stops this operation. Only the jetson repositorys should not include this file, and no coding should be done directly on the jetsons to prevent loss of work.
