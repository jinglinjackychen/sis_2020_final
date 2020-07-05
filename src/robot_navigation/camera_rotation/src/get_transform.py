#!/usr/bin/env python

import math
import time
import numpy as np
import roslib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
from camera_rotation.srv import find_plate
from tf.transformations import quaternion_matrix

class get_transform(object):
    def __init__(self):
        self.pan = 0
        self.state = 0
        self.trans = []
        self.rot = []
        self.apriltag_to_plate = np.array([[1,0,0,-0.125],[0,1,0,0],[0,0,1,0.6],[0,0,0,1]])
        self.pan_pub = rospy.Publisher("pan/command", Float64, queue_size=1)
        rospy.Service("get_pose", find_plate, self.get_pose)      
    
    def get_pose(self, req):

        tag_id = req.tag_id
        
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform("/tag_" + str(tag_id), '/camera_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if self.state == 0:
                        self.pan -= 0.1
                        self.pan_pub.publish(self.pan)
                     
                    else:
                        self.pan += 0.1
                        self.pan_pub.publish(self.pan)
                      
                    if self.pan < -0.7:
                        self.state = 1 

                    if self.pan > 0.7:
                        self.state = 0

                    time.sleep(1)
                    continue

            (trans,rot) = listener.lookupTransform("/plate_" + str(tag_id), '/camera_link', rospy.Time(0))

            pose = PoseStamped()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            print(pose)
            return ["successful", pose]

if __name__ == "__main__":
    rospy.init_node("get_transform")
    listener = tf.TransformListener()
    get_transform()
    rospy.spin()
