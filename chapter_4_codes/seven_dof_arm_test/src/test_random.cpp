#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm");
  // specify that our target will be a random one
  group.setRandomTarget();
  // plan the motion and then move the group to the sampled target 
  group.move();
  ros::waitForShutdown();
}
