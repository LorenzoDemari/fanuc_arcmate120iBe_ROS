# fanuc_arcmate120iBe_ROS
This repository contains all the files and the instructions to integrate into ROS the **FANUC ARC Mate 120iBe** manipulator, provided with the **R-J3iB** Mate controller (2003).

## Contents
This repository has the following contents:

- **binaries:** this folder contains all the ros_industrial software to be installed on the controller (.tp and .pc)
	- **binaries_source:** the source codes for the binaries
<!-- - **fanuc_arcmate120ibe_moveit_config:** a ROS package for the integration in MoveIt! of the robot
- **fanuc_arcmate120ibe_support:** a ROS package that contains the URDF of the robot and other configuration files -->
- **fanuc_command:** a ROS package that contains a few nodes to send commands to the manipulator (no planning)
- **fanuc_planning:** a ROS package that contains a few nodes to plan and execute trajectories with the manipulator
- **README:** the present document

In this readme, you will find:

1. **Requirements**

2. **Configuration:** all the instructions to configure the controller and the PC

3. **Utilization:** all the instructions to make the manipulator move through ROS

4. **fanuc_command package:** a description of the fanuc_command ROS package

## 1. Requirements

- **fanuc_driver:** https://github.com/ros-industrial/fanuc/tree/kinetic
- **industrial_robot_client:** https://github.com/ros-industrial/industrial_core
- **MoveIt!:** https://github.com/ros-planning/moveit/tree/melodic-devel

## 2. Configuration

On the **Teach Pendant** (robot console):

	- TCP/IP configuration
		- MENUS > SETUP > F1 > Next > Host Comm > TCP/IP
		- Give a name to the node (Node Name)
		- Give a Subnet Mask (255.255.255.0)
		- In Host Table insert name and IP of:
			- Robot
			- Computer

	- Software installation
		- Configure FTP Server
		- Transfer all the file in the "binaries" folder to the "md:" directory of the controller
		- Reboot the controller

	- Calibration
		- MENUS > SYSTEM > F1 > Variables
		- Find variable named "$MASTER_ENB" and set it to "1"
		- Set all the joints in the the right position
		- F1 > Master/Cal > FIXTURE POSITION MASTER
		- CALIBRATE

On a computer with **Ubuntu** and **ROS**

	- TCP/IP configuration
		- Set a static IP
		- Give the same Subnet Mask given to the robot
		
	- ROS packages
		- ros_industrial/fanuc_driver
		- ros_industrial
		- moveit
		- fanuc_arcmate120iBe_support

## 3. Utilization

On the **Teach Pendant** (robot console):

	- SELECT > F1 > TP Programs > ROS: now the code of ROS.TP will be opened
	- Press SELECT again
	- Press and keep always pressed the "Deadman switch"
	- Press RESET and wait until the light FAULT switches off
	- Press and keep always pressed SHIFT
	- Press FWD once
	- On the Teach Pendant you should see the following messages:
		- RREL Waiting for ROS traj relay
		- RSTA Waiting for ROS state prox

On a terminal in **Ubuntu**:

	- cd $FANUC_WS
	- source devel/setup.bash
	- roslaunch fanuc_arcmate120iBe_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.4
	- RViz should open with a visualization of the robot updated with the real state of the robot

On the **Teach Pendant** (robot console):

	- You should see the following messages:
		- RREL Connected
		- RSTA Connected

Now you can move the manipulator; to do it you have 2 options:

	1 - MoveIt!: 
		- move the the simulated arm in RViz from the starting position to the desired position
		- in the Planning tab, click Plan and then Plan&Execute
		- the real robot and the simulated one should reach the desired position

	2 - Publish to a topic:
		- the topic is "/joint_trajectory_action/goal"
		- the type of message is "control_msgs/FollowJointTrajectoryActionGoal.msg"
		- in the message you must initialize at least 2 configurations:
			A - the actual configuration (joint states) of the robot (you can see it subscribing to the "/joint_states" topic)
			B - the desired configuration to be reached by the manipulator
		- looking below, you can see an example of a message published on the "/joint_trajectory_action/goal" in order to move just the 5th joint
### Message Example:
![Prova](https://github.com/LorenzoDemari/fanuc_arcmate120iBe_ROS/blob/developing/message.png)

## 4. fanuc_command package

This package contains a few ROS nodes that allows the user to perform some simple tasks in an easy way.

### set_configuration
This node leads the manipulator to the configuration given as arguments to the node.

Usage:

	$ rosrun fanuc_command set_configuration.py 0.0 0.36 0.43 3.14 0.07 3.14

### set_joint_offset
This node allows to give an offset to each joint with a single message; it runs just once and then terminates.

Usage:

	$ rosrun fanuc_command set_joint_offset.py 0.0 0.0 0.0 0.0 0.0 -0.2
Running the node with these arguments, the robot will move the 6th joint of -0.2 radiants.

### set_to_zero
This node leads the robot to its default position \[0.0, 0.0, 0.0, 0.0, 0.0, 0.0\]

Usage:

	$ rosrun fanuc_command set_to_zero.py

### fanuc_keyboard
This node allows the user to teleoperate the manipulator by moving one joint at a time.

Usage:

	$ rosrun fanuc_command fanuc_keyboard.py

Commands:

	'a' JOINT_1 'd'
	'w' JOINT_2 's'
	'i' JOINT_3 'k'
	'j' JOINT_4 'l'
	'8' JOINT_5 '5'
	'4' JOINT_6 '6'
