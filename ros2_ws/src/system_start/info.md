# System start package info

The system start package launches the whole system for either the jetson or the command pc. By launching the user interface launch file the controller topic and camera streams will be started 
```
ros2 launch system_start system_user_interface_launch.py
``` 
launching all of the ROS things needed by the user side to run the full ROS system. 

While if instead the regular launch file is launched 
```
ros2 launch system_start system_launch.py
```
, everything need on the robot end of the system like camera feeds, serials, etc will be launched.

Effectively if ```ros2 launch system_start system_user_interface_launch.py``` is launched on the local controller pc, and ```ros2 launch system_start system_launch.py``` on the computer that runs the robot, the whole system needed to run and control the robot will be ran. 