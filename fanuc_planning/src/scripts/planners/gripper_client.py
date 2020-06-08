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


def gripper_control_client(x):
    rospy.wait_for_service('gripper_control')
    try:
        gripper_control = rospy.ServiceProxy('gripper_control', SetBool)
        resp1 = gripper_control(x)
        print("\n%s", resp1.message)
        # return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    gripper_control_client(1)