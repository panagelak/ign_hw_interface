# Ignition hardware interface

## Collaborators

[Christos Gkournelos](https://github.com/cgkournelos)
## Overview

Currently there is no ros control plugin for ignition gazebo, resulting in most implementations using the ignition trajectory plugin along with custom moveit action servers to implement the *follow_joint_trajectory*, omitting the dynamic nature of ros control.

Thus until the ros control plugin that will automate the construction of a hardware interface similar to gazebo classic, *a custom hardware interface approach is needed*.

**This package aims to facilitate as a template for creating this hardware interface.**

## Ignition hardware interface overview

In order to create a ros interface with ignition gazebo we will use

1. Ignition plugins for joint_state publishing
2. x6 plugins to command each joint
3. Ignition bridge to communicate with the ignition topics via ros

Then the hardware interface node will have a topic interface to communicate with ignition.

1) **read()** : the hardware interface has a separate subscriber (different thread) that subscribes to the joint state topic (joint_states_ign) coming from the ignition bridge. On the read however we fill the joint_position private value from this topic message and thus the joint_state_controller will publish the /joint_states topic. 
2) **write()** : has 1 publisher per joint that publishes the command topic

An overview of the ignition hardware interface communication can be seen below

![](demo_bringup/doc/driver_diagram.svg)

A topic interface for this purpose is not the most elegant way however it suffices for demo cases.
### Demos
![](demo_bringup/doc/ignition_moveit.gif)

### Package exploration

A quick overview of the packages

1. **demo_bringup**

    All the launch files and configuration in one package, along with test publishers and rqt multiplot configuration to tune the pids

2. **ur10_ign_description** 
    
    A modified xacro for the ur10 to be compatible with ignition gazebo (enables the ignition plugins) -> ***see_issues***

3. **ignition_hardware_interface**  

    Implements a custom Hardware interface to breach the agnostic robot ros controllers. It registers to the controller manager the Joint handles (for state and command) and then registers the Interfaces.

    Furthermore implements the required read() and write() functions. There you need to put your robot's custom API. In the current driver we have a topic interface (publisher subscriber as described in the overview)

4. **ur10_moveit_config** 
    
    Semantic description required for MoveIt + launch files to actually start move group

## Installation

### Building from Source

#### Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [Ignition Gazebo edifice](https://ignitionrobotics.org/docs/edifice/install_ubuntu)

It is recommended to use **Ubuntu 20.04 with ROS noetic**


#### Install Ignition Gazebo edifice

One can run the following commands to install the ignition gazebo binaries for edifice,
as taken from the docs [Ignition Gazebo edifice](https://ignitionrobotics.org/docs/edifice/install_ubuntu)

```bash
# Necessary tools
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
# Ignition edifice
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-edifice
```

**! You will need to also put the following export in your .bashrc**

```bash
export IGNITION_VERSION=edifice
```

#### Building procedure for ignition hardware interface

```bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone the latest version of this repository into your catkin workspace *src* folder.
git clone <repository link>

# Install dependencies of all packages.
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin_make

# activate the workspace
source devel/setup.bash

```

## How to use the Ignition Hardware Interface

The following launch file can be used

```bash
# standalone
roslaunch demo_bringup demo.launch

# with moveit and rviz
roslaunch demo_bringup demo.launch moveit:=true rviz:=true rviz_config:=moveit

# just view robot
roslaunch demo_bringup view_robot.launch
```

## How to tune the pid controllers

Currently the tune pid values (in the xacro) have been taken from this repo, and the pid values can not be dynamically reconfigure since they are loaded along with the robot_description on the ignition plugins

https://github.com/gezp/universal_robot_ign

```bash
# we need to disable the hardware interface because it keeps publishing the command topics
roslaunch demo_bringup demo.launch hardware:=false
# 6 rqt multiplots for each joint
roslaunch demo_bringup multi_plot.launch
# test publisher
rosrun demo_bringup test_pub 
```

## How to use MoveIt Servo

*Issue : currently for some reason the robot falls when it is trying to jog*

```bash
# with moveit , !moveit servo and rviz
roslaunch demo_bringup demo.launch moveit:=true moveit_servo:=true rviz:=true rviz_config:=moveit

# switch controller to arm_jog_controller
rosservice call /controller_manager/switch_controller "
start_controllers: ['arm_jog_controller']
stop_controllers: ['pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.1" 

# start teleop node (that publishes the cartesian command with keyboard input)

# give it permissions
roscd demo_bringup/scripts && chmod +x cartesian_teleop.py

# run it
rosrun demo_bringup cartesian_teleop.py

# to go back to trajectory controller 
rosservice call /controller_manager/switch_controller "
start_controllers: ['pos_joint_traj_controller']
stop_controllers: ['arm_jog_controller']
strictness: 2
start_asap: false
timeout: 0.1" 
```

### How to use the teleop node

! if you press a button it will keep publishing press l to stop
! only one button will be taken into account each time

q, w, e, r, t, y -> +lin.x, +lin.y, +lin.z, +ang.x, +ang.y, +ang.z respectively

a, s, d, f, g, h -> -lin.x, -lin.y, -lin.z, -ang.x, -ang.y, -ang.z respectively

i : increase speed, k : decrease speed, l : stop movement

## urdf issues

1. The node that spawns the robot in ignition based of the *robot_description* parameter has issues with colored dae files showing the robot without color
2. The format to load the mesh files needs to be the following
  a) *file://$(find ur10_ign_description)* instead of *package://ur10_ign_description*
3. gazebo classic does not accept this format, however rviz/robot_state_publisher is ok with both.
4. This issue can be overcomed if you load the robot as a model in ignition and load the robot_description separately for ros.
## Resources

https://ignitionrobotics.org/docs/edifice
https://github.com/PickNikRobotics/ros_control_boilerplate
https://github.com/gezp/universal_robot_ign