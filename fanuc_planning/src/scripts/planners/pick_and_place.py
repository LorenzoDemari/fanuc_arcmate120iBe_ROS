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
from std_srvs.srv import SetBool
from std_msgs.msg import Float64


def range_callback(msg):
    global distance
    distance = msg.data


def gripper_control_client(x):
    rospy.wait_for_service('gripper_control')
    try:
        gripper_control = rospy.ServiceProxy('gripper_control', SetBool)
        resp1 = gripper_control(x)
        print("\n%s", resp1.message)
        # return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def pick_place(ox, oy, oz, w, x, y, z):
    global distance
    global plan_0
    ''' STAGE I : Move the EE near the object of interest'''
    pose_target = geometry_msgs.msg.Pose()
    # Coordinates and orientaion of the desired pose
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = w
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    # Build such Pose as a target for aour planning group
    move_group.set_pose_target(pose_target)
    # Plan the trajectory
    plan_0 = move_group.go(wait=True)
    print "============ Fanuc 120iBe: OVER THE OBJECT"
    #
    #
    #
    #
    #
    # # ''' STAGE II : Move the EE just over the object of interest'''
    # pose_target.position.z = 0.7  # This is in relation to joint_6 NOT END-EFFECTOR!!
    # # Build such Pose as a target for aour planning group
    # move_group.set_pose_target(pose_target)
    # # Plan the trajectory
    # plan_0 = move_group.go(wait=True)
    # print "============ Fanuc 120iBe: APPROACHING OBJECT"
    # gripper_control_client(1)

    print "============ Fanuc 120iBe: APPROACHING OBJECT"
    while (distance > 6):
        pose_target.position.z = pose_target.position.z - 0.05
        # Build such Pose as a target for aour planning group
        move_group.set_pose_target(pose_target)
        # Plan the trajectory
        plan_0 = move_group.go(wait=True)
        print "============ Distance: %f", distance

    gripper_control_client(1)
    #
    #
    # rospy.sleep(2) # Slowing down the execution
    #
    #
    # ''' STAGE III : Grasp the object of interest'''
    # '''
    # touch_links = robot.get_link_names(group=grasping_group)
    # scene.attach_box(eef_link, box_name, touch_links=touch_links)
    # print "============ Fanuc 120iBe: GRASPING OBJECT"
    # '''
    #
    # rospy.sleep(2) # Slowing down the execution
    #
    #
    # ''' STAGE IV : Lift the object of interest '''
    pose_target.position.z = 1  # This is in relation to joint_6 NOT END-EFFECTOR!!
    # Build such Pose as a target for aour planning group
    move_group.set_pose_target(pose_target)
    # Plan the trajectory
    plan_0 = move_group.go(wait=True)
    print "============ Fanuc 120iBe: LIFTING OBJECT"
    #
    #
    #
    #
    #
    ''' STAGE V : Move the object of interest over its desired location '''
    pose_target.position.y = 0.5  # This is in relation to joint_6 NOT END-EFFECTOR!!
    # Build such Pose as a target for aour planning group
    move_group.set_pose_target(pose_target)
    # Plan the trajectory
    plan_0 = move_group.go(wait=True)
    print "============ Fanuc 120iBe: MOVING OBJECT"
    #
    #
    #
    #
    #
    ''' STAGE VI : Approach the object of interest closer to its new location'''
    pose_target.position.z = 0.5  # This is in relation to joint_6 NOT END-EFFECTOR!!
    # Build such Pose as a target for aour planning group
    move_group.set_pose_target(pose_target)
    # Plan the trajectory
    plan_0 = move_group.go(wait=True)
    print "============ Fanuc 120iBe: APPROACHING GOAL"
    gripper_control_client(0)
    #
    #
    # rospy.sleep(2) # Slowing down the execution
    #
    #
    # ''' STAGE VII :  Release the object'''
    # print "============ Fanuc 120iBe: RELEASING OBJECT"
    # scene.remove_attached_object(eef_link, box_name)
    #
    #
    # rospy.sleep(2) # Slowing down the execution
    #
    #
    # ''' STAGE VII : Leave the object'''
    # '''
    pose_target.position.z = 1  # This is in relation to joint_6 NOT END-EFFECTOR!!
    # Build such Pose as a target for aour planning group
    move_group.set_pose_target(pose_target)
    # Plan the trajectory
    plan_0 = move_group.go(wait=True)
    print "============ Fanuc 120iBe: LEAVING OBJECT"
    #
    #
    #
    #
    #
    #




''' Initializing the node representing the robot '''
moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('fanuc_move_group_pick_and_place', anonymous=True)

distance = 100.0
rospy.Subscriber('range_sensor_data', Float64, range_callback)

# Provides information such as the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Interface for getting, setting, and updating the robot's understanding of the world
scene = moveit_commander.PlanningSceneInterface()

# Interface to a planning group (group of joints). Used to plan and execute motions
group_name = "fanuc_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# We can also print the name of the end-effector link for this group
eef_link = move_group.get_end_effector_link()

# Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)




'''PICK and PLACE parameters of interest'''
scale = 1
waypoints = [] # Can plan a Cartesian path directly by specifying a list of waypoints
box_name = 'cube' # Name of the object to be grasped
grasping_group = 'magnet' # Name of the group containing the end-effector



''' STAGE O : Move the whole arm to its initial pose'''
move_group.set_named_target("origin") # Known position defined throught the MoveIt setup assistant
plan_0 = move_group.go(wait=True)
print "============ Fanuc 120iBe: READY"
#print('\x1b[6;31;43m' + '============ Fanuc 120iBe:' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'READY' + " " +  '\x1b[0m' )
move_group.stop()

rx = -0.7071
ry = -0.7071
rz = 0
rw = 0
# sphere 1
pick_place(rx, ry, rz, rw, 1.19, -0.265, 1.3)
# sphere 2
pick_place(rx, ry, rz, rw, 1.19, 0.265, 1.3)
# # sphere 3
pick_place(rx, ry, rz, rw, 0.81, -0.265, 1.3)
# # sphere 4
pick_place(rx, ry, rz, rw, 0.81, 0.265, 1.3)
# # sphere 5
pick_place(rx, ry, rz, rw, 1, 0, 1.3)


''' STAGE IX : go back to initial position'''
move_group.set_named_target("origin") # Known position defined throught the MoveIt setup assistant
plan_0 = move_group.go(wait=True)
print "============ Fanuc 120iBe: POSITION RESET"
#
#
#
#
#''' STAGE X : Shuting everything down'''
move_group.stop()
rospy.sleep(2)
move_group.clear_pose_targets() 
print "============ Fanuc 120iBe: OFF"
moveit_commander.roscpp_shutdown()