# fanuc_arcmate120iBe_ROS
This repository contains all the files and the instructions to integrate the FANUC ARC Mate 120iBe provided with the R-J3iB Mate controller (2003) within ROS.
In this readme, you will find all the instructions to configure the controller and the PC (software to be installed and configuration to be done) and to make the manipulator move through ROS.

# CONFIGURATION

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
