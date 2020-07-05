#!/usr/bin/env python

import numpy as np
import rospy
import rospkg
from tf import transformations as tr
from pozyx_ros.msg import DeviceRange
from pozyx_ros.msg import DeviceRangeArray
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import math
import csv

class RangesSave(object):
    """docstring for RangesSave."""
    def __init__(self):
        super(RangesSave, self).__init__()

        veh = rospy.get_param("~veh")
        round = rospy.get_param("~round")
        print(veh, round)

        ## parameters
        self.filename = rospkg.RosPack().get_path('pozyx_ros')+"/result/"+veh+"_"+round
        self.anchor = int(rospy.get_param("~anchor_id"))

        ## variable
        self.ranges = {}

        ## subscribers
        pozyx_sub_bl = rospy.Subscriber('back_left/ranges', DeviceRangeArray, self.ranges_cb, ('back_left'),queue_size=1)
        pozyx_sub_br = rospy.Subscriber('back_right/ranges', DeviceRangeArray, self.ranges_cb, ('back_right'),queue_size=1)
        pozyx_sub_fl = rospy.Subscriber('front_left/ranges', DeviceRangeArray, self.ranges_cb, ('front_left'),queue_size=1)
        pozyx_sub_fr = rospy.Subscriber('front_right/ranges', DeviceRangeArray, self.ranges_cb, ('front_right'),queue_size=1)

        ## save ranges trigger
        trigger_s = rospy.Service('save_trigger', Trigger, self.dump_data)

    def ranges_cb(self, ranges_msg, position):

        for anchor in ranges_msg.rangeArray:
            if anchor.tag_id == self.anchor and anchor.distance != 0:
                if position not in self.ranges:
                    self.ranges[position] = np.array([])
                self.ranges[position] = np.append(self.ranges[position], anchor.distance)

    def dump_data(self, req):
        
        for loc in self.ranges:
            filename = self.filename + '_' + loc + '.csv'
            np.savetxt(self.filename, self.ranges[loc], delimiter=",")
        
        return TriggerResponse(
            success = True,
            message = 'file saved'
        )

if __name__ == '__main__':
	rospy.init_node('ranges_save_csv',anonymous=False)
	s = RangesSave()
	rospy.spin()
