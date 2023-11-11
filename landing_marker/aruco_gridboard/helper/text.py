#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os 

odom_string = None

def string_callback(msg):
    global odom_string
    # Callback function for std_msgs/String topic
    file_name = os.path.basename(msg.data)
    # print(file_name)
    # print(odom_string)
    with open('/home/analys/Documents/rtk.txt', 'a') as file:
      if odom_string is not None :
        file.write(file_name+odom_string)

def odom_callback(msg):
    # print(msg)
    # Callback function for geometry_msgs/Odometry topic
    global odom_string
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular
    odom_string = f",{pos.x},{pos.y},{pos.z}"
    odom_string += f",{ori.w},{ori.x},{ori.y},{ori.z}"
    odom_string += f",{lin.x},{lin.y},{lin.z}"
    odom_string += f",{ang.x},{ang.y},{ang.z}\n"
    # print (odom_string)

def main():
    rospy.init_node('subscriber_node', anonymous=True)

    # Subscribe to std_msgs/String topic
    rospy.Subscriber('/image_name', String, string_callback)

    # Subscribe to geometry_msgs/Odometry topic
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)

    # Spin ROS
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass