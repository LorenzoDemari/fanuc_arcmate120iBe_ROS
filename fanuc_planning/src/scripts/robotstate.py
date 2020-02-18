#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Initializing the node representing the robot
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('fanuc_move_group', anonymous=True)

# Provides information such as the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Provides a remote interface for getting, setting, and updating the robot's internal understanding of the
# surrounding world:
scene = moveit_commander.PlanningSceneInterface()

# Is an interface to a planning group (group of joints).Used to plan and execute motions
group_name = "fanuc_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

##==================================##
###     INFORMATION OF INTEREST    ###
##==================================##
# The name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# The name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# Retrieving the list of all the groups in the robot (config package):
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# The entire state of the robot
print "============ Printing robot state"
print robot.get_current_state()
print ""

print "============ Printing robot pose"
print move_group.get_current_pose().pose
print ""
moveit_commander.roscpp_shutdown()