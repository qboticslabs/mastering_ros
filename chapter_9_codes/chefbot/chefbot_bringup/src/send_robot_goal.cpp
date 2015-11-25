#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_goals");

	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	try{
		goal.target_pose.pose.position.x = atof(argv[1]);
		goal.target_pose.pose.position.y = atof(argv[2]);
		goal.target_pose.pose.orientation.w = atof(argv[3]);
	   }
	catch(int e){


		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;


	}
	ROS_INFO("Sending move base goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Robot has arrived to the goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	return 0;
}
