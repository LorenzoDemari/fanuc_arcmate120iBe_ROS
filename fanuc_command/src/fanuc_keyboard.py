#!/usr/bin/env python
import fcntl
import termios

import actionlib
import numpy as np
import rospy
import sys, os
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from sensor_msgs.msg import JointState

moveBindings = {
        'a': (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        'd': (-0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        'w': (0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
        's': (0.0, -0.1, 0.0, 0.0, 0.0, 0.0),
        'i': (0.0, 0.0, -0.1, 0.0, 0.0, 0.0),
        'k': (0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        'j': (0.0, 0.0, 0.0, 0.1, 0.0, 0.0),
        'l': (0.0, 0.0, 0.0, -0.1, 0.0, 0.0),
        '8': (0.0, 0.0, 0.0, 0.0, 0.1, 0.0),
        '5': (0.0, 0.0, 0.0, 0.0, -0.1, 0.0),
        '4': (0.0, 0.0, 0.0, 0.0, 0.0, 0.1),
        '6': (0.0, 0.0, 0.0, 0.0, 0.0, -0.1),
    }


def state_callback(state_msg):
    global actual_config
    global flag
    actual_config = state_msg.position
    # rospy.loginfo(actual_config)
    flag = True


def keyboard():
    global flag
    global actual_config
    flag = False

    fd = sys.stdin.fileno()
    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    rospy.init_node('joint_keyboard', anonymous=True)
    # client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    # client.wait_for_server()
    # rospy.loginfo("SERVER")

    # pub1 = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, state_callback)
    pub = rospy.Publisher('/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    # rospy.sleep(5)
    rate = rospy.Rate(1)  # 10hz
    try:
        while not rospy.is_shutdown():
            if flag:
                # Starting configuration initialization based on the joint state stored by state_callback
                start_config = JointTrajectoryPoint()
                start_config.positions = actual_config
                start_config.time_from_start.secs = 0
                start_config.time_from_start.nsecs = 0
                try:
                    c = sys.stdin.read(1)
                    if c in moveBindings.keys():
                        print c
                        offsets = moveBindings[c]
                        print "OFFSETS: ", offsets

                        # Goal configuration initialization
                        goal_config = JointTrajectoryPoint()
                        goal_config_array = np.asarray(actual_config)
                        for i in range(0,6):
                            goal_config_array[i] = goal_config_array[i] + offsets[i]
                            print goal_config_array[i]
                        print "\n"
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

                        # client.send_goal(traj_msg)
                        # client.wait_for_result()
                        # return client.get_result()

                        # rospy.spin()
                        rate.sleep()
                    else:
                        print "Wrong key selected!!"
                        pass

                except IOError:
                    pass
            else:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)


if __name__ == '__main__':
    try:
        keyboard()
    except rospy.ROSInterruptException:
        pass
