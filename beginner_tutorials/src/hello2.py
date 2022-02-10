#!/usr/bin/env python3
import rospy

rospy.init_node('hello_node')   # initialize a node called hello_node

while not rospy.is_shutdown():
    rospy.loginfo('Hello World')    # print Hello World in terminal
    rospy.sleep(1)