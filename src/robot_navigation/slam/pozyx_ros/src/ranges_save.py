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

class RangesSave(object):
    """docstring for RangesSave."""
    def __init__(self):
        super(RangesSave, self).__init__()

        veh = rospy.get_param("~veh")
        round = rospy.get_param("~round")
        print(veh, round)

        ## parameters
        self.filename = rospkg.RosPack().get_path('pozyx_ros')+"/result/"+veh+"_"+round+'.yaml'
        self.anchors = [0x674f, 0x6742, 0x6a1c, 0x6755, 0x6a22,0x6705]

        ## variable
        self.ranges = {}
        self.start_time = None
        self.start_time_switch = True
        self.start_id = None

        ## subscribers
        pozyx_sub_bl = rospy.Subscriber('back_left/ranges', DeviceRangeArray, self.ranges_cb, ('back_left'),queue_size=1)
        pozyx_sub_br = rospy.Subscriber('back_right/ranges', DeviceRangeArray, self.ranges_cb, ('back_right'),queue_size=1)
        pozyx_sub_fl = rospy.Subscriber('front_left/ranges', DeviceRangeArray, self.ranges_cb, ('front_left'),queue_size=1)
        pozyx_sub_fr = rospy.Subscriber('front_right/ranges', DeviceRangeArray, self.ranges_cb, ('front_right'),queue_size=1)

        ## save ranges trigger
        trigger_s = rospy.Service('save_trigger', Trigger, self.dump_data)

    def ranges_cb(self, ranges_msg, position):
    
        # if any topics gets the first msg
        if self.start_time_switch:
            self.start_time_switch =False
            self.start_time = ranges_msg.header.stamp
            self.start_id = (self.start_time.secs % 1000000)*10 + self.start_time.nsecs / 100000000

        # If any topics gets its first topic
        if position not in self.ranges:
            self.ranges[position] = {}

        
        # Append until the length equal to this_id
        this_time = ranges_msg.header.stamp
        this_id = (this_time.secs % 1000000)*10 + this_time.nsecs / 100000000
        this_id = this_id - self.start_id
        for tag_id in self.anchors:
            if tag_id not in self.ranges[position]:
                self.ranges[position][tag_id] = np.array([])
            delta_t = this_id + 1 - len(self.ranges[position][tag_id])
            self.ranges[position][tag_id] = np.append(self.ranges[position][tag_id], np.zeros((delta_t),dtype=int))

        for devices in ranges_msg.rangeArray:
            if len(self.ranges[position][devices.tag_id]) != (this_id + 1):
                rospy.logerr("The id and length are not match!!!")
            self.ranges[position][devices.tag_id][-1] = devices.distance

    def dump_data(self, req):
        # print("Save to file.")

        save_string = ''
        save_string += 'start_time:\n'
        save_string += '  secs: ' + str(self.start_time.secs) + '\n'
        save_string += '  nsecs: ' + str(self.start_time.nsecs) + '\n'
        save_string += 'data:\n'

        # distance data
        for position in self.ranges:
            save_string += '  ' +　position + ':\n'
            for device_id in self.ranges[position]:
                save_string += '    ' + str(device_id) + ':\n'
                for r in self.ranges[position][device_id]:
                    save_string += '      - ' + str(r) + '\n'
        
        f = open(self.filename, 'w')
        f.write(save_string)
        f.close()
        
        return TriggerResponse(
            success = True,
            message = 'file saved'
        )

if __name__ == '__main__':
	rospy.init_node('ranges_save',anonymous=False)
	s = RangesSave()
	rospy.spin()
