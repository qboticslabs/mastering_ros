#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "remove_collision_objct");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // We obtain the current planning scene and wait until everything is up
    // and running, otherwise the request won't succeed
    moveit::planning_interface::PlanningSceneInterface current_scene;

    sleep(5.0);

    // We add the name of the objects we want to remove into the vector and
    // use the planning scene interface to remove those collision objects
    std::vector<std::string> object_ids;
    object_ids.push_back("seven_dof_arm_cylinder");
    current_scene.removeCollisionObjects(object_ids);

    ros::shutdown();

    return 0;
}
