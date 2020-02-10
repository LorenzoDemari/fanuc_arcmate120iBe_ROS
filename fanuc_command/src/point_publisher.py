#!/usr/bin/env python
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Duration
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryActionGoal

import actionlib
import actionlib_msgs.msg
import industrial_msgs.srv._CmdJointTrajectory
from industrial_msgs.msg import _RobotMode

def state_callback(state_msg):
    global actual_config
    global flag
    actual_config = state_msg.position
    rospy.loginfo(actual_config)
    flag = True

def command():
    global flag
    global actual_config
    flag = False
    rospy.init_node('joint_commander', anonymous=True)
    # client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    # client.wait_for_server()
    # rospy.loginfo("SERVER")

    # pub1 = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, state_callback)
    pub = rospy.Publisher('/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rospy.sleep(5)

    if flag:
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():

        # Starting start_config initialization based on the joint state stored by state_callback
        start_config = JointTrajectoryPoint()
        # start_config.positions = [-0.24934397637844086, 1.1815193891525269, -0.9878455400466919, 0.0027158225420862436, 0.7455053925514221, -0.7225106954574585]
        start_config.positions = actual_config
        # start_config.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # start_config.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        start_config.time_from_start.secs = 0
        start_config.time_from_start.nsecs = 0

        goal_config = JointTrajectoryPoint()
        # goal_config.positions = [-0.24934397637844086, 1.1815193891525269, -0.9878455400466919, 0.0027158225420862436, 0.5455053925514221, -0.7225106954574585]
        goal_config_array = np.asarray(actual_config)
        goal_config_array[4] = goal_config_array[4] + 0.4
        goal_config.positions = goal_config_array
        # goal_config.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # goal_config.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
            # rate.sleep()
    else:
        pass


if __name__ == '__main__':
    try:
        command()
    except rospy.ROSInterruptException:
        pass

