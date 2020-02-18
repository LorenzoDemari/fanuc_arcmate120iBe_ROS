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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


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

# Origin (Known position)
# move_group.set_named_target("origin") # Known position defined throught the MoveIt setup assistant
origin = Pose()
origin.position.x = 0.990
origin.position.y = 0.08
origin.position.z = 1.395
origin.orientation.x = 0.5
origin.orientation.y = 0.5
origin.orientation.z = 0.5
origin.orientation.w = 0.5

##===== PLANNING TO A GOAL POSE =====##

goal_pose = geometry_msgs.msg.Pose()
goal_pose.orientation.x = 0.7
goal_pose.orientation.y = 0.7
goal_pose.orientation.z = 0
goal_pose.orientation.w = 0
goal_pose.position.x = 0.9
goal_pose.position.y = -0.3
goal_pose.position.z = 0.9

move_group.set_pose_target(goal_pose)
print(move_group.get_joint_value_target())
print(move_group.get_planning_time())
print(move_group.get_random_joint_values())

## Now, we call the planner to compute the plan and execute it.
plan = move_group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
move_group.stop()

# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()
move_group.set_named_target("origin")
plan_0 = move_group.go(wait=True)

moveit_commander.roscpp_shutdown()


''' '''