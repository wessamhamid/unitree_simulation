# Unitree Quadruped Robot Gazebo Simulation (Aliengo, A1 & Go1)
This repository contains all the files and code needed to simulate the [Aliengo](https://www.unitree.com/aliengo/), [A1](https://www.unitree.com/products/a1) & [Go1](https://www.unitree.com/products/go1) quadruped robots using [Gazebo](http://gazebosim.org/) and [ROS](https://www.ros.org/).
The software runs on [ROS noetic](http://wiki.ros.org/noetic) and [Ubuntu 20.04](http://www.releases.ubuntu.com/20.04/). 

<p align="center">
	<img src="./resources/aliengo_walk.gif" height="400">
</p>


## Setup
### Install Dependencies 
```
sudo apt-get install ros-$ROS_DISTRO-controller-manager \
	ros-$ROS_DISTRO-joint-state-publisher-gui \
	ros-$ROS_DISTRO-joint-state-controller \
	ros-$ROS_DISTRO-teleop-twist-keyboard \
	ros-$ROS_DISTRO-position-controllers \
	ros-$ROS_DISTRO-gazebo-ros-control \
	ros-$ROS_DISTRO-effort-controllers \
	ros-$ROS_DISTRO-teleop-twist-joy \
	ros-$ROS_DISTRO-gazebo-ros-pkgs \
	ros-$ROS_DISTRO-twist-mux \
	ros-$ROS_DISTRO-xacro
```

### Workspace Setup
```
mkdir catkin_ws && cd catkin_ws
mkdir src && cd src
git clone https://github.com/wessamhamid/unitree_simulation.git
cd unitree_simulation
chmod +x unitree_joystick/scripts/ramped_joystick.py
chmod +x quadruped_unitree/scripts/*.py
cd controllers
chmod +x aliengo_controller/scripts/robot_controller_gazebo.py
chmod +x a1_controller/scripts/robot_controller_gazebo.py
chmod +x go1_controller/scripts/robot_controller_gazebo.py
cd ..
cd ..
cd ..
catkin_make
source devel/setup.bash
```

## Run Simulation

### Simulation 1 - Robot Description: Control individual joints.

For Aliengo:
```
roslaunch aliengo_description aliengo_rviz.launch
```

For A1:
```
roslaunch a1_description a1_rviz.launch
```

For Go1:
```
roslaunch go1_description go1_rviz.launch
```

<p align="center">
    <img src="./resources/go1_desc.png" height="200">
	<img src="./resources/a1_desc.png" height="200">
	<img src="./resources/aliengo_desc.png" height="200">
</p>

### Simulation 2 - Walking teleoperation within Gazebo.

#### Terminal 1 - Launch Gazebo models:
For Aliengo:

```
roslaunch quadruped_unitree aliengo_gazebo.launch
```
For A1:
```
roslaunch quadruped_unitree a1_gazebo.launch
```

For Go1:
```
roslaunch quadruped_unitree go1_gazebo.launch
```

#### Terminal 2 - Teleoperation using keyboard:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/key_vel
```


#### Note:
If robot falls over, press `ctrl + shift + r` while in Gazebo window to reset robot.


<p align="center">
    <img src="./resources/go1_pcl.png" height="250">
	<img src="./resources/go1_walk.gif" height="320">
</p>

<p align="center">
    
</p>

### Worlds
#### To change the Gazebo world for Simulation 2:
```
roslaunch quadruped_unitree aliengo_gazebo.launch world:=normal
```
Replace `normal` with any of the following worlds:
- `earth`
- `office`
- `space` 

#### Setting Up Additional Worlds (from AWS):
```
cd catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
cd ../
catkin_make --only-pkg-with-deps aws_robomaker_small_warehouse_world
source devel/setup.bash
```

Launch the simulation:
```
roslaunch quadruped_unitree go1_gazebo.launch world:=no_roof_small_warehouse
```

<p align="center">
    <img src="./resources/go1_aws.png" height="300">
</p>

## Credits
 - lnotspotl: https://github.com/lnotspotl/a1_sim_py
 - kimsooyoung: https://github.com/kimsooyoung/a1_sim_py
 - mike4192: https://github.com/mike4192/spotMicro
 - Unitree Robotics: https://github.com/unitreerobotics/a1_ros
 - QUADRUPED ROBOTICS: https://quadruped.de
