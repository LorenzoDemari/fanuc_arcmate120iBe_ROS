#!/usr/bin/env python
import rospy
import csv
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
    with open('data.csv', 'a') as csvfile:
        spamwriter = csv.writer(csvfile, delimiter=' ')    #, quotechar='|', quoting=csv.QUOTE_MINIMAL)
        data_str = str(data.data)
        data_str = data_str.replace('(', '')
        data_str = data_str.replace(')', '')
        data_str = data_str.replace(' ', '')
        rospy.loginfo("%s", data_str)
        spamwriter.writerow(str(rospy.get_rostime()) + ',' + data_str)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    with open('data.csv', 'w') as csvfile:
        headerwriter = csv.writer(csvfile, delimiter=' ')
        headerwriter.writerow('ROS_Time' + ',' + '1st_Sensor' + ',' + '2nd_Sensor' + ',' + '3rd_Sensor' + ',' +
                              '4th_Sensor' + ',' + '5th_Sensor' + ',' + '6th_Sensor' + ',' + 'Norm')
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", Float64MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
