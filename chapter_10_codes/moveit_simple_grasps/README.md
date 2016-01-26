MoveIt! Simple Grasps
====================

A basic grasp generator for simple objects such as blocks or cylinders for use with the MoveIt! pick and place pipeline. Does not consider friction cones or other dynamics. 

Its current implementation simple takes as input a pose vector (postition and orientation) and generates a large number of potential grasp approaches and directions. Also includes a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

This package includes:

 - Simple pose-based grasp generator for a block
 - Separate grasp generators for custom objects such as rectanguar or cylindrical objects
 - Grasp filter
 - Test code and visualizations

Developed by [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder with outside contributors.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_simple_grasps/hydro-devel/resources/demo.png" /> 

## Video Demo

A simple demo with Baxter:

[![Baxter Grasp Test](http://img.youtube.com/vi/WEDITCR2qH4/0.jpg)](https://www.youtube.com/watch?v=WEDITCR2qH4)  

## Build Status

[![Build Status](https://travis-ci.org/davetcoleman/moveit_simple_grasps.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/moveit_simple_grasps)

## Install

### Ubuntu Debian

Hydro:
```
sudo apt-get install ros-hydro-moveit-simple-grasps
```
Indigo:
```
sudo apt-get install ros-indigo-moveit-simple-grasps
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:

Hydro:
```
rosdep install --from-paths src --ignore-src --rosdistro hydro
```
Indigo:
```
rosdep install --from-paths src --ignore-src --rosdistro indigo
```

## Robot-Agnostic Configuration

You will first need a configuration file that described your robot's end effector geometry. Currently an example format can be seen in this repository at [config/baxter_grasp_data.yaml](https://github.com/davetcoleman/moveit_simple_grasps/blob/hydro-devel/config/baxter_grasp_data.yaml). See the comments within that file for explanations. 

To load that file at launch, you copy the example in the file [launch/grasp_test.launch](https://github.com/davetcoleman/moveit_simple_grasps/blob/hydro-devel/launch/grasp_test.launch) where you should see the line ``<rosparam command="load" file="$(find moveit_simple_grasps)/config/baxter_grasp_data.yaml"/>``.

## Code Usage

Note: You might find the moveit_blocks.h example, discussed at the bottom of this page, most helpful.

We will discuss how to use the generation, filtering, and visualization components.

Within your robot's ROS package, add this package to your package.xml, CMakeLists.txt. Then in whatever C++ file add this to your includes:
```
// Grasp generation and visualization
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
```

Add to your class's member variables the following:
```
// Grasp generator
moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

// class for publishing stuff to rviz
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

// robot-specific data for generating grasps
moveit_simple_grasps::GraspData grasp_data_;
```

In your class' constructor initialize the visualization tools;
```
// Load the Robot Viz Tools for publishing to Rviz
visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
```
Change the first parameter of visual tools to the name of your robot's base link. For more information on that package, see [moveit_visual_tools](https://github.com/davetcoleman/moveit_visual_tools).

Then load your robot's custom .yaml grasp data file:
```
// Load grasp data specific to our robot
ros::NodeHandle nh("~");
if (!grasp_data_.loadRobotGraspData(nh, "left_hand"))
  ros::shutdown();
```
Where "left_hand" is the name of one your SRDF-defined MoveIt! end effectors from the Setup Assistant. This data is loaded from a file that you must load to the parameter server within a roslaunch file, as desribed above.

Now load grasp generator:
```
// Load grasp generator
simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
```

To generate grasps, you first need the pose of the object you want to grasp, such as a block. Here's an example pose:
```
geometry_msgs::Pose object_pose;
object_pose.position.x = 0.4;
object_pose.position.y = -0.2;
object_pose.position.z = 0.0;

// Orientation
double angle = M_PI / 1.5;
Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
object_pose.orientation.x = quat.x();
object_pose.orientation.y = quat.y();
object_pose.orientation.z = quat.z();
object_pose.orientation.w = quat.w();
```

If you want to visualize this object pose as a block:
```
visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, 0.04);
```

Now generate the grasps:
```
std::vector<moveit_msgs::Grasp> possible_grasps;
simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);
```

To visualize:
```
visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
```

## Grasp Filter Usage

This component creates several threads and tests a large number of potential grasps for kinematic feasibility.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_simple_grasps/hydro-devel/resources/filter.png" />

To filter grasps after generating them:
```
// Filter the grasp for only the ones that are reachable
bool filter_pregrasps = true;
std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);
```

To view the filtered grasps along with the planning group pose:
```
visual_tools_->publishIKSolutions(ik_solutions, planning_group_name_, 0.25);
```

There is more that is undocumented but I'm tired of writing this.

## Tested Robots

 - [Baxter](https://github.com/davetcoleman/baxter_cpp)
 - [REEM](http://wiki.ros.org/Robots/REEM)

## Example Code

A new (still in development) example tool is ``moveit_blocks.h`` located in the ``include`` folder. It gives you a complete pick and place pipeline using this package and MoveIt, and all you need is the appropriate config file and launch file. An example launch file can be found [here](https://github.com/davetcoleman/clam/blob/master/clam_pick_place/launch/pick_place.launch).

There are currently example implementations:

 - [baxter_pick_place](https://github.com/davetcoleman/baxter_cpp/tree/hydro-devel/baxter_pick_place)
 - [reem_tabletop_grasping](https://github.com/pal-robotics/reem_tabletop_grasping)
 - [clam_pick_place](https://github.com/davetcoleman/clam/tree/master/clam_pick_place)

## Testing

There are two tests scripts in this package. To view the tests, first start Rviz with:

```
roslaunch moveit_simple_grasps grasp_test_rviz.launch
```

To test just grasp generation for randomly placed blocks:
```
roslaunch moveit_simple_grasps grasp_test.launch 
```

To also test the IK grasp filtering:
```
roslaunch moveit_simple_grasps grasp_filter_test.launch
```

## TODO

Features we'd like to see added to this project:

 - Ability to reason about any shape, not just centroid of a bounding box
   - Input arbitrary meshes
   - Auto create a bounding box around that mesh
 - Better reasoning about support surfaces (table)
 - Integrate collision checking to verify feasibility of grasp
 - Support non-parallel gripper end effectors
 - Make grasp quality metric better informed
 - Make this project easier to setup for new robots
   - Integrate into Setup Assistant GUI
 - Improve simple pick and place pipline header file

## Contributors

 - Dave Coleman, CU Boulder @davetcoleman
 - Bence Magyar, PAL Robotics @bmagyar