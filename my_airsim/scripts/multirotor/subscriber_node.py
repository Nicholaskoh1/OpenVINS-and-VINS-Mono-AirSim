#!/usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Odometry

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("Drone coordinates: ({}, {}, {})".format(x, y, z))


rospy.init_node('subscriber_node')
rospy.Subscriber('/vins_estimator/camera_pose', Odometry, odom_callback)
rospy.spin()

