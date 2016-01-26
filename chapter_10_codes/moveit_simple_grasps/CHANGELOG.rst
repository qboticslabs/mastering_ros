^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_simple_grasps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2014-10-27)
------------------
* Refactored for new moveit_visual_tools API
* Fixed package.xml
* Updated README
* Contributors: Dave Coleman

1.2.0 (2014-09-19)
------------------

1.1.0 (2014-07-31)
------------------
* Fixed grasp pose rotation
* Created new verbose constructor flag to enable easy debugging
* Allow a grasp pose to be rotated along z axis
* Created new pick place pipeline template
* Moved ClamArm config to this repo
* Updated package description
* Updated README
* Contributors: Dave Coleman

1.0.1 (2014-05-30)
------------------
* Moved base link out of individual end effector configurations
* Fixed tests for new gripper config format
* Fix for strict cppcheck and g++ warnings/errors
* Remove self assignment
* fix functions with no return statement and other cppcheck errors fix
* Compatibility changes for ROS Indigo (Eigen find pkg)
* Restored the lost grasp data for REEM
* Renamed grasp data config file for REEM and updated launch file accordingly.
* Fix lost contents of file and add left hand.
* Enabled dual arm grasping, filtering
* Fixes for more strict moveit_visual_tools data access
* Deprecated function, made robot grasp config files have more than 1 end effector
* Fixed posture bug and renamed local vars to not have _ postfix
* Added ability to filter pre-grasps as well
* Removed this-> because does not follow MoveIt style guidelines
* Made tests left/right invariant
* Refactored RobotGraspData and loader function
* Renamed RobotGraspData to GraspData
* Made graspDataLoader into a function the GraspData class.
* Move grasp data struct to separate file
* Fixed filter test
* Created left hand config for baxter
* Improved tests
* Changed deprecated function name
* Add options to grasp generation action goal.
* Yaml conversion
* Improved error handling of loading from yaml, removed unnecessary data
* Fixes for new grasp data loader
* Fixes per catkin_lintg
* Convert baxter_data.h to baxter_gripper.yaml
* Add launch script for grasp generator server + testing node
* Replace reem_data.h with reem_hand.yaml
* Add grasp_data_loader to be used by server.
* Changed way visualizations are made
* Moved the visualize grasp functionality to moveit_visual_tools. Deprecated generateAllGrasps
* Fixed grasp filter bug of pose in wrong frame of reference
* Trying to visualize arm reaches
* Fixed grasp filter
* Added new hand_roll feature
* Changed name of moveit_simple_grasps
* Renamed files and classes to not have the word MoveIt
* Added picture
* Added ability to do full grasp rotation, finer grained access, documentation
* Fix travis
* Changed name of moveit_visual_tools
* Initial commit
* Contributors: Bence Magyar, Dave Coleman, Jordi Pages
