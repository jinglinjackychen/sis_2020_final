#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>

#define PI 3.14159

using namespace cv;
using namespace std;

int point_position[2], pineapple_position[2];
double orientation_angle = 0;


void get_image_mask(Mat mask_image)
{
	Mat mask_image_rgb(mask_image.rows, mask_image.cols, CV_8UC3, Scalar(0)), pick(mask_image.rows, mask_image.cols, CV_8UC1, Scalar(0));
	int i, j;

	for (i = 0; i < mask_image.rows; i++){
		for(j = 0; j < mask_image.cols; j++){
			printf("%d ", mask_image.at<uchar>(i, j, 0));
			if(mask_image.at<uchar>(i, j, 0) == 0){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 0;
			}else if(mask_image.at<uchar>(i, j, 0) == 1){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 255;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 0;
			}else if(mask_image.at<uchar>(i, j, 0) == 2){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 255;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 0;
			}else if(mask_image.at<uchar>(i, j, 0) == 3){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 4){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 0;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 255;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 255;
			}else if(mask_image.at<uchar>(i, j, 0) == 5){
				mask_image_rgb.at<Vec3b>(i, j)[0] = 255;
				mask_image_rgb.at<Vec3b>(i, j)[1] = 255;
				mask_image_rgb.at<Vec3b>(i, j)[2] = 255;
			}

		}
	}

	imwrite("/home/sis/sis_competition_2020/catkin_ws/src/mask1.jpg",mask_image_rgb);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "take_mask");
	ros::NodeHandle n;

	/*if (prediction_client.call(prediction_req)){
		ROS_INFO("plan_result: %s", prediction_req.response.info.c_str());
	}
	else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}*/
	Mat mask;
	Mat mask_channel[3];
	mask = imread("/home/sis/sis_competition_2020/catkin_ws/src/origin_mask.jpg",1);
	split(mask,mask_channel);

	//get_object_position(prediction_req.response.mask);
        //orientation_angle = get_object_orientation();
	get_image_mask(mask_channel[1]);

	ros::spinOnce();
	return 0;
}
