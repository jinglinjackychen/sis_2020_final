#!/usr/bin/env python

import sys
import rospy
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest

def	to_pos_client(arg_pose):
	rospy.wait_for_service('to_position')
	try:
		to_position = rospy.ServiceProxy('to_position', GoToPos)
		request = GoToPosRequest()
		request.pos = arg_pose
		response = to_position(request)
		return response.result
	except rospy.ServiceException, e:
		print "Service call failed : %s"%e

if __name__ == "__main__":
	if len(sys.argv) == 2:
		arg_pose = int(sys.argv[1])
	else:
		print "no arg_pose"
	to_pos_client(arg_pose)