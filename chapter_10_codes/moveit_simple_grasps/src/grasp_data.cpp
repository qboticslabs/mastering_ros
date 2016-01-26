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

#include <moveit_simple_grasps/grasp_data.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace moveit_simple_grasps
{
GraspData::GraspData() :
  // Fill in default values where possible:
  base_link_("/base_link"),
  grasp_depth_(0.12),
  angle_resolution_(16),
  approach_retreat_desired_dist_(0.6),
  approach_retreat_min_dist_(0.4),
  object_size_(0.04)
{}

bool GraspData::loadRobotGraspData(const ros::NodeHandle& nh, const std::string& end_effector)
{
  std::vector<std::string> joint_names;
  std::vector<double> pre_grasp_posture; // todo: remove all underscore post-fixes
  std::vector<double> grasp_posture;
  std::vector<double> grasp_pose_to_eef;
  std::vector<double> grasp_pose_to_eef_rotation;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;
  std::string end_effector_parent_link;

  // Load a param
  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `base_link` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam("base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  // Load a param
  if (!child_nh.hasParam("pregrasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `pregrasp_time_from_start` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << child_nh.getNamespace());
    return false;
  }
  child_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

  // Load a param
  if (!child_nh.hasParam("grasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_time_from_start` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("grasp_time_from_start", grasp_time_from_start);

  // Load a param
  if (!child_nh.hasParam("end_effector_name"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `end_effector_name` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_name", end_effector_name);

  // Load a param
  if (!child_nh.hasParam("end_effector_parent_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `end_effector_parent_link` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_parent_link", end_effector_parent_link);

  // Load a param
  if (!child_nh.hasParam("joints"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `joints` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  XmlRpc::XmlRpcValue joint_list;
  child_nh.getParam("joints", joint_list);
  if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    for (int32_t i = 0; i < joint_list.size(); ++i)
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      joint_names.push_back(static_cast<std::string>(joint_list[i]));
    }
  else
    ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???");

  if(child_nh.hasParam("pregrasp_posture"))
  {
    XmlRpc::XmlRpcValue preg_posture_list;
    child_nh.getParam("pregrasp_posture", preg_posture_list);
    ROS_ASSERT(preg_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < preg_posture_list.size(); ++i)
    {
      ROS_ASSERT(preg_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pre_grasp_posture.push_back(static_cast<double>(preg_posture_list[i]));
    }
  }

  ROS_ASSERT(child_nh.hasParam("grasp_posture"));
  XmlRpc::XmlRpcValue grasp_posture_list;
  child_nh.getParam("grasp_posture", grasp_posture_list);
  ROS_ASSERT(grasp_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < grasp_posture_list.size(); ++i)
  {
    ROS_ASSERT(grasp_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    grasp_posture.push_back(static_cast<double>(grasp_posture_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef"));
  XmlRpc::XmlRpcValue g_to_eef_list;
  child_nh.getParam("grasp_pose_to_eef", g_to_eef_list);
  ROS_ASSERT(g_to_eef_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < g_to_eef_list.size(); ++i)
  {
    // Cast to double OR int
    if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef` wrong data type - int or double required.");
        return false;
      }
      else
        grasp_pose_to_eef.push_back(static_cast<int>(g_to_eef_list[i]));
    }
    else
      grasp_pose_to_eef.push_back(static_cast<double>(g_to_eef_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef_rotation"));
  XmlRpc::XmlRpcValue g_to_eef_rotation_list;
  child_nh.getParam("grasp_pose_to_eef_rotation", g_to_eef_rotation_list);
  ROS_ASSERT(g_to_eef_rotation_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < g_to_eef_rotation_list.size(); ++i)
  {
    // Cast to double OR int
    if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef_rotation` wrong data type - int or double required.");
        return false;
      }
      else
        grasp_pose_to_eef_rotation.push_back(static_cast<int>(g_to_eef_rotation_list[i]));
    }
    else
      grasp_pose_to_eef_rotation.push_back(static_cast<double>(g_to_eef_rotation_list[i]));
  }

  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short

  // Orientation
  ROS_ASSERT(grasp_pose_to_eef_rotation.size() == 3);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(grasp_pose_to_eef_rotation[1]), Eigen::Vector3d::UnitY())); // turn on Z axis
  // TODO: rotate for roll and yaw also, not just pitch (unit y)
  // but i don't need that feature right now and it might be tricky
  grasp_pose_to_eef_pose_.orientation.x = quat.x();
  grasp_pose_to_eef_pose_.orientation.y = quat.y();
  grasp_pose_to_eef_pose_.orientation.z = quat.z();
  grasp_pose_to_eef_pose_.orientation.w = quat.w();

  // Position // approach vector?
  ROS_ASSERT(grasp_pose_to_eef.size() == 3);
  grasp_pose_to_eef_pose_.position.x = grasp_pose_to_eef[0];
  grasp_pose_to_eef_pose_.position.y = grasp_pose_to_eef[1];
  grasp_pose_to_eef_pose_.position.z = grasp_pose_to_eef[2];

  // -------------------------------
  // Create pre-grasp posture if specified
  if(!pre_grasp_posture.empty())
  {
    pre_grasp_posture_.header.frame_id = base_link_;
    pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    pre_grasp_posture_.joint_names = joint_names;
    // Position of joints
    pre_grasp_posture_.points.resize(1);
    pre_grasp_posture_.points[0].positions = pre_grasp_posture;
    pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
  }
  // -------------------------------
  // Create grasp posture
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start);

  // -------------------------------
  // SRDF Info
  ee_parent_link_ = end_effector_parent_link;
  ee_group_ = end_effector_name;

  // -------------------------------
  // Nums
  approach_retreat_desired_dist_ = 0.2; // 0.3;
  approach_retreat_min_dist_ = 0.06;
  // distance from center point of object to end effector
  grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other side of the object! (like from below)

  // generate grasps at PI/angle_resolution increments
  angle_resolution_ = 16; //TODO parametrize this, or move to action interface

  // Debug
  //moveit_simple_grasps::SimpleGrasps::printObjectGraspData(grasp_data);

  return true;
}

bool GraspData::setRobotStatePreGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, pre_grasp_posture_ );
}
bool GraspData::setRobotStateGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, grasp_posture_ );
}

bool GraspData::setRobotState( robot_state::RobotStatePtr &robot_state, const trajectory_msgs::JointTrajectory &posture )
{
  // Do for every joint in end effector
  for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
  {
    // Debug
    std::cout << "Setting joint " << posture.joint_names[i] << " to value " 
              << posture.points[i].positions[0] << std::endl;

    // Set joint position
    robot_state->setJointPositions( posture.joint_names[i],
                                    posture.points[i].positions );
  }
}

void GraspData::print()
{
  ROS_WARN_STREAM_NAMED("grasp_data","Debug Grasp Data variable values:");
  std::cout << "grasp_pose_to_eef_pose_: \n" <<grasp_pose_to_eef_pose_<<std::endl;
  std::cout << "pre_grasp_posture_: \n" <<pre_grasp_posture_<<std::endl;
  std::cout << "grasp_posture_: \n" <<grasp_posture_<<std::endl;
  std::cout << "base_link_: " <<base_link_<<std::endl;
  std::cout << "ee_parent_link_: " <<ee_parent_link_<<std::endl;
  std::cout << "ee_group_: " <<ee_group_<<std::endl;
  std::cout << "grasp_depth_: " <<grasp_depth_<<std::endl;
  std::cout << "angle_resolution_: " <<angle_resolution_<<std::endl;
  std::cout << "approach_retreat_desired_dist_: " <<approach_retreat_desired_dist_<<std::endl;
  std::cout << "approach_retreat_min_dist_: " <<approach_retreat_min_dist_<<std::endl;
  std::cout << "object_size_: " <<object_size_<<std::endl;
}

} // namespace
