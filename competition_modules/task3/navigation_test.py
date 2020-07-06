#!/usr/bin/env python

import sys
import rospy
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from camera_rotation.srv import find_plate, find_plateResponse, find_plateRequest


def navigation(x):
    #print("hello1")
    rospy.wait_for_service('to_position')

    try :
        navigation_client = rospy.ServiceProxy('to_position', GoToPos)
        command = GoToPosRequest()
        command.pos = x
        response = navigation_client(command)
	#print("hello7")
        return response.result

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
	

def navigation2(y):
    #print("hello2")
    rospy.wait_for_service('get_pose')

    try :
        navigation_client2 = rospy.ServiceProxy('get_pose', find_plate)
        command2 = find_plateRequest()      
        command2.plate = y    
        response2 = navigation_client2(command2)
	#print("hello8") 
	return response2.result
            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("out")
        sys.exit(1)
    navigation(x)
    navigation2(y)
    
