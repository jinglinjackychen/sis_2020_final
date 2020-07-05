#!/usr/bin/python

import rospy
import numpy as np
import cv2  # OpenCV module
import time

from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("detection",anonymous=True)

output_result = rospy.Publisher('/camera/color/image_result', Image, queue_size=10)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def nothing(x):
    pass

def rosImageCallback(msg):


    cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    blurred_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
	
	# detect green
    lower_green = np.array([52, 135, 55])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

	# detect blue
    lower_blue = np.array([89, 127, 61])
    upper_blue = np.array([168, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    mask = mask_red + mask_green + mask_blue

    result = cv2.bitwise_and (cv_image, cv_image, mask=mask)	

    output_result.publish(cv_bridge.cv2_to_imgmsg(result, encoding="passthrough"))

if __name__=="__main__":

    rospy.Subscriber('/camera/color/image_rect_color', Image, rosImageCallback)
    rospy.spin()