#!/usr/bin/env python
import numpy as np
import rospy
import sys
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from sensor_msgs.msg import JointState


def state_callback(state_msg):
    global actual_config
    global flag
    actual_config = state_msg.position
    rospy.loginfo(actual_config)
    flag = True

def command():
    if len(sys.argv) != 7:
        print "Usage: ", sys.argv[0], "<angle_joint_1> <angle_joint_2> <angle_joint_3> <angle_joint_4> <angle_joint_5> <angle_joint_6>"
        return -1
    else:
        global flag
        global actual_config
        flag = False
        rospy.init_node('configuration_commander', anonymous=True)
        # rospy.loginfo("SERVER")

        rospy.Subscriber("/joint_states", JointState, state_callback)
        pub = rospy.Publisher('/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        rospy.sleep(1)

        if flag:

            # Starting configuration initialization based on the joint state stored by state_callback
            start_config = JointTrajectoryPoint()
            start_config.positions = actual_config
            start_config.time_from_start.secs = 0
            start_config.time_from_start.nsecs = 0

            # Goal configuration initialization
            goal_config = JointTrajectoryPoint()
            goal_config_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for i in range(0,6):
                goal_config_array[i] = float(sys.argv[i+1])
                print goal_config_array[i]
            goal_config.positions = goal_config_array
            goal_config.time_from_start.secs = 1
            goal_config.time_from_start.nsecs = 63310604

            # Message inizialization
            traj_msg = FollowJointTrajectoryActionGoal()
            traj_msg.goal.trajectory.header.seq = 0
            traj_msg.goal.trajectory.header.stamp.nsecs = 0
            traj_msg.goal.trajectory.header.stamp.secs = 0
            traj_msg.goal.trajectory.header.frame_id = 'base_link'
            traj_msg.goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            traj_msg.goal.trajectory.points = [start_config, goal_config]
            traj_msg.goal.goal_time_tolerance.secs = 0
            traj_msg.goal.goal_time_tolerance.nsecs = 0
            # rospy.loginfo(goal_config.positions[1])

            # Message pubblication on "/joint_trajectory_action/goal"
            pub.publish(traj_msg)

            flag = False

        else:
            pass


if __name__ == '__main__':
    try:
        command()
    except rospy.ROSInterruptException:
        pass
