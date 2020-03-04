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
from geometry_msgs.msg import Pose

# Initializing the node representing the robot
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('fanuc_move_group_cartesian', anonymous=True)

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




## You can plan a Cartesian path directly by specifying a list of waypoints
## for the end-effector to go through. If executing  interactively in a
## Python shell, set scale = 1.0.

##current_pose = move_group.get_current_pose().pose


scale = 1
waypoints = [] # Can plan a Cartesian path directly by specifying a list of waypoints

wpose = move_group.get_current_pose().pose
wpose.position.x -= scale * 0.5  # First move up (z)
waypoints.append(copy.deepcopy(wpose))
#wpose.position.x += scale * 0.05  # First move up (z)
#waypoints.append(copy.deepcopy(wpose))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

#move_group.go(wait=True)
move_group.execute(plan)


# Calling `stop()` ensures that there is no residual movement
move_group.stop()

# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()

moveit_commander.roscpp_shutdown()

                                   