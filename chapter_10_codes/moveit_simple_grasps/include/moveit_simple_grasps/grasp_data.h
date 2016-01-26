/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Univ of CO, Boulder, PAL Robotics, S.L.
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: Bence Magyar, Dave Coleman
   Description: Data class used by the grasp generator.
*/

#ifndef MOVEIT_SIMPLE_GRASPS__GRASP_DATA_H_
#define MOVEIT_SIMPLE_GRASPS__GRASP_DATA_H_

// Ros
#include <ros/node_handle.h>

// Msgs
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_simple_grasps
{

class GraspData
{
public:
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  trajectory_msgs::JointTrajectory pre_grasp_posture_; // when the end effector is in "open" position
  trajectory_msgs::JointTrajectory grasp_posture_; // when the end effector is in "close" position
  std::string base_link_; // name of global frame with z pointing up
  std::string ee_parent_link_; // the last link in the kinematic chain before the end effector, e.g. "/gripper_roll_link"
  std::string ee_group_; // the end effector name
  double grasp_depth_; // distance from center point of object to end effector
  int angle_resolution_; // generate grasps at PI/angle_resolution increments
  double approach_retreat_desired_dist_; // how far back from the grasp position the pregrasp phase should be
  double approach_retreat_min_dist_; // how far back from the grasp position the pregrasp phase should be at minimum
  double object_size_; // for visualization

public:

  /**
   * \brief Constructor
   */
  GraspData();

  /**
   * \brief Loads grasp data from a yaml file (load from roslaunch)
   * \param node handle - allows for namespacing
   * \param end effector name - which side of a two handed robot to load data for. should correspond to SRDF EE names
   * \return true on success
   */
  bool loadRobotGraspData(const ros::NodeHandle& nh, const std::string& end_effector);

  /**
   * \brief Alter a robot state so that the end effector corresponding to this grasp data is in pre-grasp state (OPEN)
   * \param joint state of robot
   * \return true on success
   */
  bool setRobotStatePreGrasp( robot_state::RobotStatePtr &robot_state );

  /**
   * \brief Alter a robot state so that the end effector corresponding to this grasp data is in grasp state (CLOSED)
   * \param joint state of robot
   * \return true on success
   */
  bool setRobotStateGrasp( robot_state::RobotStatePtr &robot_state );

  /**
   * \brief Alter a robot state so that the end effector corresponding to this grasp data is in a grasp posture
   * \param joint state of robot
   * \param posture - what state to set the end effector
   * \return true on success
   */
  bool setRobotState( robot_state::RobotStatePtr &robot_state, const trajectory_msgs::JointTrajectory &posture );

  /**
   * \brief Debug data to console
   */
  void print();
};

} // namespace

#endif
