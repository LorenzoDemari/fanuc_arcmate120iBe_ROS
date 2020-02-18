#! /usr/bin/env python
import rospy
import tf
import math
import numpy as np
import std_msgs.msg
import moveit_msgs.msg
import trajectory_msgs.msg
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory

''' '''
def kinematics_callback(traj_msg):
    global joint_pos_config
    global joint_vel_config
    #global joint_acc_config
    global flag
    joint_pos_config = traj_msg.trajectory.joint_trajectory.points.positions
    joint_vel_config = traj_msg.trajectory.joint_trajectory.points.velocities
    #joint_acc_config = traj_msg.trajectory.joint_trajectory.points.accelerations
    flag = True





def main():
    rospy.init_node('trajectory_listener', anonymous=False)
    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, kinematics_callback())
    pub = rospy.Publisher('trajectory/waypoints/joints_positions', JointTrajectory, queue_size=1)

    global joint_pos_config
    global joint_vel_config
    #global joint_acc_config
    global flag

    rospy.sleep(5)

    if flag:
        goal_joint_pos_config = np.asarray(joint_pos_config)
        goal_joint_vel_config = np.asarray(joint_vel_config)
        #goal_joint_acc_config = np.asarray(joint_vel_config)

        goal_joint_trajectory = RobotTrajectory()
        #goal_joint_trajectory.joint_trajectory.header.seq = 0
        #goal_joint_trajectory.joint_trajectory.header.stamp.secs = 0
        #goal_joint_trajectory.joint_trajectory.header.stamp.nsecs = 0
        #goal_joint_trajectory.joint_trajectory.header.frame_id = 'world'
        goal_joint_trajectory.joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        goal_joint_trajectory.joint_trajectory.points.positions = goal_joint_pos_config
        goal_joint_trajectory.joint_trajectory.points.velocities = goal_joint_vel_config
        #joint_trajectory.points.accelerations = goal_joint_acc_config
        #joint_trajectory.points.effort = []
        #joint_trajectory.points.time_from_start.secs = 0
        #joint_trajectory.points.time_from_start.secs = 0 
        print(goal_joint_trajectory)
        pub.publish(goal_joint_trajectory)

        flag = False
    else:
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

'''TO-DO:
Extraer la trajectoria
'''