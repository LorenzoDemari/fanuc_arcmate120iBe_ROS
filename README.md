# fanuc_arcmate120iBe_ROS
This repository contains all the files and the instructions to integrate the FANUC ARC Mate 120iBe provided with the R-J3iB Mate controller (2003) within ROS.
In this readme, you will find all the instructions to configure the controller and the PC (software to be installed and configuration to be done) and to make the manipulator move through ROS.

# Configuration

On the Teach Pendant (robot console):

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

On a computer with Ubuntu and ROS

	- TCP/IP configuration
		- Set a static IP
		- Give the same Subnet Mask given to the robot
		
	- ROS packages
		- ros_industrial/fanuc_driver
		- ros_industrial
		- moveit
		- fanuc_arcmate120iBe_support

# Utilization

On the Teach Pendant (robot console):

	- SELECT > F1 > TP Programs > ROS: now the code of ROS.TP will be opened
	- Press SELECT again
	- Press and keep always pressed the "Deadman switch"
	- Press RESET and wait untile the light FAULT switches off
	- Press and keep always pressed SHIFT
	- Press FWD once
	- On the Teach Pendant you should see the following messages:
		- RREL Waiting for ROS traj relay
		- RSTA Waiting for ROS state prox

On a terminal in Ubuntu:

	- cd $FANUC_WS
	- source devel/setup.bash
	- roslaunch fanuc_$ROBOTMODEL_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.4
	- RViz should open with a visualization of the robot updated with the real state of the robot

On the Teach Pendant (robot console):

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
		- the type of message is "control_msgs/FollowJointTrajectoryActionGoal"
		- in the message you must initialize at least 2 configurations:
			A - the actual configuration (joint states) of the robot (you can see it subscribing to the "/joint_states" topic)
			B - the desired configuration to be reached by the manipulator
		- looking below, you can see an example of a message published on the "/joint_trajectory_action/goal" in order to move just the 5th joint

# Message Example:
![Prova](https://github.com/LorenzoDemari/fanuc_arcmate120iBe_ROS/blob/developing/message.png)
