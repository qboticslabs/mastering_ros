#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;

ros::Time current_time_encoder_l, last_time_encoder_l;
ros::Time current_time_encoder_r, last_time_encoder_r;

double DistancePerCount = (3.14159265 * 0.45) / 2100;

double x = 0;
double y = 0;
double th = 0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;
double deltaLeft;
double deltaRight;


void left_encoder_Cb(const std_msgs::Int64::ConstPtr& left_ticks)
{
	current_time_encoder_l = ros::Time::now();
	deltaLeft = left_ticks->data - _PreviousLeftEncoderCounts;
	vx = deltaLeft * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();
	_PreviousLeftEncoderCounts = left_ticks->data;
	last_time_encoder_l = current_time_encoder_l;

//	ROS_INFO_STREAM("Left encoder" << left_ticks->data);

}

void right_encoder_Cb(const std_msgs::Int64::ConstPtr& right_ticks)
{
	current_time_encoder_r = ros::Time::now();
	deltaRight = right_ticks->data - _PreviousRightEncoderCounts;
	vy = deltaRight * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();
	_PreviousRightEncoderCounts = right_ticks->data;
	last_time_encoder_r = current_time_encoder_r;

//	ROS_INFO_STREAM("Right encoder" << right_ticks->data);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  ROS_INFO("Starting Wheel Odometry Node");
  ros::Subscriber sub_l = n.subscribe("lwheel", 100, left_encoder_Cb);
  ros::Subscriber sub_r = n.subscribe("rwheel", 100, right_encoder_Cb);


  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);   
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(1.0);
  while(n.ok()){

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    ros::spinOnce();  
    r.sleep();
  }
}

