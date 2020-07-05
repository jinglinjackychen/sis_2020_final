#!/usr/bin/env python

import rospy

rospy.init_node("hello_world")

while not rospy.is_shutdown():

    rospy.loginfo("hello world")
    rospy.sleep(1)

