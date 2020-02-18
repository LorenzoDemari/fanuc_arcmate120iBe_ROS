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
from geometry_msgs.msg import Pose, PoseStamped

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


rospy.sleep(2)
scene.remove_world_object()
p = PoseStamped()



'''SETUP'''
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0
p.pose.position.y = 0
p.pose.position.z = -0.05
scene.add_box("platform", p, (1.2, 1.2, 0.1))

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = -0.65
p.pose.position.y = 0.
p.pose.position.z = 0.2625
scene.add_box("box_electronics_1", p, (0.65, 0.65,0.525))



''' OBSTACLES'''
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0
p.pose.position.y = 1.4
p.pose.position.z = 1
scene.add_box("wall_1", p, (3.05, 0.1, 2))
#scene.remove_world_object("wall_1")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = -1.47
p.pose.position.y = 0
p.pose.position.z = 1
scene.add_box("wall_2", p, (0.1, 2.7, 2))
#scene.remove_world_object("wall_2")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 1
p.pose.position.y = 0
p.pose.position.z = 0.9
scene.add_box("panel", p, (0.3, 0.1, 0.5))
scene.remove_world_object("panel")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = -0.5
p.pose.position.y = -1
p.pose.position.z = 1
scene.add_box("wall_3", p, (2, 0.1, 2))
#scene.remove_world_object("wall_3")



'''WORKSPACE'''
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 1
p.pose.position.y = 0
p.pose.position.z = 0.6
scene.add_box("table", p, (0.7, 1.4, 0.05))
scene.remove_world_object("table")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.7
p.pose.position.y = -0.65
p.pose.position.z = 0.25
scene.add_box("leg_1", p, (0.08, 0.08, 0.7))
scene.remove_world_object("leg_1")


p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.7
p.pose.position.y = 0.65
p.pose.position.z = 0.25
scene.add_box("leg_2", p, (0.08, 0.08, 0.7))
scene.remove_world_object("leg_2")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 1.3
p.pose.position.y = -0.65
p.pose.position.z = 0.25
scene.add_box("leg_3", p, (0.08, 0.08, 0.7))
scene.remove_world_object("leg_3")

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 1.3
p.pose.position.y = 0.65
p.pose.position.z = 0.25
scene.add_box("leg_4", p, (0.08, 0.08, 0.7))
scene.remove_world_object("leg_4")



'''OBJECTS'''
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 1
p.pose.position.y = -0.5
p.pose.position.z = 0.7
scene.add_box("cube", p, (0.1, 0.1, 0.1))
scene.remove_world_object("cube")