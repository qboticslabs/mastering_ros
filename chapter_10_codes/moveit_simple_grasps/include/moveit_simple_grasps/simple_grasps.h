/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/

// Author: Dave Coleman
// Desc:   Generates grasps for a cube

#ifndef MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_H_
#define MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_H_

// ROS
#include <ros/ros.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

#include <moveit_simple_grasps/grasp_data.h>

namespace moveit_simple_grasps
{

static const double RAD2DEG = 57.2957795;

// Grasp axis orientation
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_direction_t {UP, DOWN};
enum grasp_rotation_t {FULL, HALF};

// Class
class SimpleGrasps
{
private:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Transform from frame of box to global frame
  Eigen::Affine3d object_global_transform_;

  // Display more output both in console and in Rviz (with arrows and markers)
  bool verbose_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles). With GCC, this is done with a attribute ((aligned(16))).

  /**
   * \brief Constructor
   */
  SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr rviz_tools, bool verbose = false);

  /**
   * \brief Destructor
   */
  ~SimpleGrasps();

  /**
   * \brief Moved to generateBlockGrasps
   */
  MOVEIT_DEPRECATED bool generateAllGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    generateBlockGrasps(object_pose, grasp_data, possible_grasps);

    return true;
  }

  /**
   * \brief Create all possible grasp positions for a block
   * \param pose of block, where vector arrow is parallel to table plane
   * \param data describing end effector
   * \param resulting generated possible grasps
   * \return true if successful
   */ 
  bool generateBlockGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Create grasp positions in one axis around a single pose
   *        Note: to visualize these grasps use moveit_visual_tools.publishAnimatedGrasps() function or
   *        moveit_visual_tools.publishIKSolutions() with the resulting data
   * \param pose - center point of object to be grasped
   * \param axis - axis relative to object pose to rotate generated grasps around
   * \param direction - a parallel gripper is typically symetric such that it can perform the same grasp 
   *                    180 degree around. this option allows to generate a flipped grasp pose
   * \param rotation - amount to rotate around the object - 180 or 360 degrees
   * \param hand_roll - amount in radians to roll wrist with respect to center point of object during grasp. use 0 by default
   * \param grasp_data - parameters specific to the robot geometry
   * \param possible_grasps - the output solution vector of possible grasps to attempt. ok if pre-populated
   * \return true if successful
   */
  bool generateAxisGrasps(
    const geometry_msgs::Pose& object_pose,
    grasp_axis_t axis,
    grasp_direction_t direction,
    grasp_rotation_t rotation,
    double hand_roll,
    const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Using an input grasp description, get the pregrasp pose
   * \param grasp description
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Print debug info
   * DEPRECATRED: moved to grasp_data.cpp
   */
  MOVEIT_DEPRECATED static void printObjectGraspData(const GraspData& data)
  {
    ROS_INFO_STREAM_NAMED("grasp","ROBOT GRASP DATA DEBUG OUTPUT ---------------------");
    ROS_INFO_STREAM_NAMED("grasp","Base Link: " << data.base_link_);
    ROS_INFO_STREAM_NAMED("grasp","EE Parent Link: " << data.ee_parent_link_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Depth: " << data.grasp_depth_);
    ROS_INFO_STREAM_NAMED("grasp","Angle Resolution: " << data.angle_resolution_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Desired Dist: " << data.approach_retreat_desired_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Min Dist: " << data.approach_retreat_min_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Pregrasp Posture: \n" << data.pre_grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Posture: \n" << data.grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","---------------------------------------------------\n");
  }

}; // end of class

typedef boost::shared_ptr<SimpleGrasps> SimpleGraspsPtr;
typedef boost::shared_ptr<const SimpleGrasps> SimpleGraspsConstPtr;

} // namespace

#endif
