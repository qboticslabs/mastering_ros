/*
wheel_odometry.cpp : C++ implementation of diff_tf.py node in 
http://wiki.ros.org/differential_drive
Author : Lentin Joseph
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>

#include <math.h>
#include <stdio.h>

using namespace std;

class Diff_Tf
{

public:
	Diff_Tf();



	

protected:
	ros::NodeHandle n;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;
	


	void left_encoder_Cb(const std_msgs::Int64::ConstPtr& left_ticks);

	void right_encoder_Cb(const std_msgs::Int64::ConstPtr& right_ticks);

};

//Class definition of DiffTf
Diff_Tf::Diff_Tf(){

	ROS_INFO("Starting Encoder to TF node"); 

	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	l_wheel_sub = n.subscribe("/lwheel",10, &Diff_Tf::left_encoder_Cb, this);
	
	r_wheel_sub = n.subscribe("rwheel",10, &Diff_Tf::right_encoder_Cb,this);

	
}

//Left encoder callback

void Diff_Tf::left_encoder_Cb(const std_msgs::Int64::ConstPtr& left_ticks)

{

 ROS_INFO_STREAM("Left tick" << left_ticks->data);

}


//Right encoder callback

void Diff_Tf::right_encoder_Cb(const std_msgs::Int64::ConstPtr& right_ticks)

{
;

 ROS_INFO_STREAM("Right tick" << right_ticks->data);

}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
	Diff_Tf obj;

	ros::spin();

	return 0;		
	

}
