# Linux tools
## General tools
**List all files hidden and unhidden in folder**
```
ls -a
```

**Create new directory**
```
mkdir test
```

**Create/edit file**
```
nano file.txt
```

**Make file execuatable**
```
chmod +x file.py
```

## System services
**List running system services**
```
systemctl list-units --type=service
```

**Start system service**
```
sudo systemctl daemon-reload
sudo systemctl enable heartbeat.service
sudo systemctl start heartbeat.service
```

**Check status of system service**
```
sudo systemctl status heartbeat.service
```

**Check service startup errors/warnings, use (warning, err) to see warnings/errors, remove (-p err) to see both**
```
journalctl -b -u heartbeat.service -p err
```

## Video tools
**List /dev cameras**
```
v4l2-ctl --list-devices
```

**Usful tool to visualy inspect cameras**
```
qv4l2
```

# ROS tools

**A simple command which stops ROS from running**
```
ros2 daemon stop
```

**Advenced tool to kill all ROS operations, this operation breaks ROS so system will need to be cleaned**
```
pkill -f ros2
rm -rf /dev/shm/fastrtps_*
rm -rf /dev/shm/ros2_*
source /opt/ros/humble/setup.bash
source ~/Documents/NMT-Lunabotics2025-Python-based/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

**List ros nodes and topics**
```
ros2 node list
```

```
ros2 topic list
```

**List ros topic data usage**
```
ros2 topic bw /cmd_vel
```

**List all ROS parameters of a node/topic**
```
ros2 param list /controller_server
```

**Clean and rebuild ros workspace**
```
rm -rf build/ log/ install/
colcon build --symlink-install
source install/setup.bash
```

# Arduino tools
**Recompile and uplude sketch and clean workspace**
```
arduino-cli compile --fqbn arduino:avr:mega --clean system_control/
python3 system_control/uploud_code.py
```

arduino-cli compile --fqbn arduino:avr:mega --clean actuator_calibrate/
python3 actuator_calibrate/uploud_code.py




# Extra
**Launch nav2**
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=Documents/NMT-Lunabotics2025-Python-based/ros2_ws/src/point_navigation/config/nav2_params.yaml

**Launch serial**
ros2 run serial_commands serial_talker_node.py --ros-args -p navigation_mode:=true -p log_level:=info

**Launch point navigator**
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=ros2_ws/src/point_navigation/config/nav2_params.yaml

**Launch rviz2 with config**
rviz2 -d ros2_ws/src/point_navigation/config/nav.rviz

**Set system waypoint**
ros2 param set /waypoint target_name berm
ros2 service call /save_target_location std_srvs/srv/SetBool "{data: true}"

**Launch apriltag node with visuals**
ros2 run camera camera_apriltag_node.py ros2 run camera camera_apriltag_node --ros-args -p publish_map:=true -p publish_processed:=true








cd ~/Documents/NMT-Lunabotics2025-Python-based/ros2_ws
colcon build --symlink-install
source install/setup.bash








ros2 service call /rtabmap/rtabmap/reset std_srvs/srv/Empty

ros2 service call /rtabmap/reset_odom std_srvs/srv/Empty

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}"