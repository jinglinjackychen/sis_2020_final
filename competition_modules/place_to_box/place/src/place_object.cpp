#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include "place_to_box/data.h"
#define PI 3.14159

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "to_place_object");
	ros::NodeHandle nh;
        ros::Publisher tilt_publisher = nh.advertise<std_msgs::Float64> ("/tilt/command", 20);
	ros::ServiceClient client_place = nh.serviceClient<place_to_box::data>("place");
	place_to_box::data pose;
	std_msgs::Float64 tilt_angle;


	pose.request.pose.position.x = 0.4;
	pose.request.pose.position.y = 0.0;
	pose.request.pose.position.z = 0.1;
	pose.request.pose.orientation.x = 0.0;
	pose.request.pose.orientation.y = 0.707;
	pose.request.pose.orientation.z = 0.0;
	pose.request.pose.orientation.w = 0.707;
	pose.request.id = 0;

	tilt_angle.data = 0.0;

	for(int i=0;i<20;i++){
		tilt_publisher.publish(tilt_angle);
	}

	if(client_place.call(pose)){
		cout << "\nobj_id: \n" << pose.response.obj_id;
	}

	ros::spinOnce();
	return 0;
}
