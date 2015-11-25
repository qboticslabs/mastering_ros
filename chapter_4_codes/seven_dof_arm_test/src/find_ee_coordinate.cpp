/* Seven Dof arm planning code */

/* Author : Lentin Joseph */
 

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)

{

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);



	spinner.start();

	moveit::planning_interface::MoveGroup group("arm");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;

	/*Retrive position and orientation */
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
	std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

	



	ROS_INFO("Reference frame : %s",group.getPlanningFrame().c_str());

	ROS_INFO("Reference frame : %s",group.getEndEffectorLink().c_str());
	
	
	sleep(4.0);

	ros::shutdown();
//	return 0;




}
