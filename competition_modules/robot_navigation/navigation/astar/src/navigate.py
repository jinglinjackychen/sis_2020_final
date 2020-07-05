#!/usr/bin/env python

import rospy
from pure_pursuit import PurePursuit
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from visualization_msgs.msg import Marker


class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())
        self.end = [[0.1, 2.2],
                    [4.1, 3.8],
                    [4.1, 2.2],
                    [4.1, 0.5]]
        self.pursuit = PurePursuit()
        self.pursuit.set_look_ahead_distance(0.2)
        self.pose = PoseStamped()
        self.pose.pose.position.x = self.end[0][0]
        self.pose.pose.position.y = self.end[0][1]
        self.pose.pose.position.z = 0

        self.pub_goal_marker = rospy.Publisher(
            "goal_marker", Marker, queue_size=1)
        self.pub_pid_goal = rospy.Publisher(
            "pid_control/goal", PoseStamped, queue_size=1)
        self.req_path_srv = rospy.ServiceProxy("plan_service", GetPlan)
        self.sub_pose = rospy.Subscriber(
            "pose", PoseStamped, self.cb_pose, queue_size=1)
        self.srv_topos = rospy.Service("to_position", GoToPos, self.to_pos)

    def pub_marker(self, goal=Marker()):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "look ahead"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.id = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.pub_goal_marker.publish(marker)

    def to_pos(self, req):
        rospy.loginfo("%s : request pos %d" % (rospy.get_name(), req.pos))
        res = GoToPosResponse()

        if req.pos < 0 or req.pos > 3:
            rospy.logerr("%s : pos not exist" % rospy.get_name())
            res.result = False
            return res
        if self.pose is None:
            rospy.logerr("%s : no pose" % rospy.get_name())
            res.result = False
            return res

        end_p = PoseStamped()
        end_p.pose.position.x = self.end[req.pos][0]
        end_p.pose.position.y = self.end[req.pos][1]
        end_p.pose.position.z = 0
        req_path = GetPlanRequest()
        req_path.start = self.pose
        req_path.goal = end_p
        try:
            res_path = self.req_path_srv(req_path)
        except:
            rospy.logerr("%s : path request fail" % rospy.get_name())
            res.result = False
            return res

        self.pursuit.set_path(res_path.plan)

        count = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            goal = self.pursuit.get_goal(self.pose)
            if goal is None:
                break
            self.pub_marker(goal)
            self.pub_pid_goal.publish(goal)

            count += 1
            if count > 5:
                break

        res.result = True
        return res

    def cb_pose(self, msg):
        self.pose = msg


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav = Navigation()
    rospy.spin()
