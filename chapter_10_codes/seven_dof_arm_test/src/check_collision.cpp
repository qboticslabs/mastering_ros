/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


// Collision Checking
// ^^^^^^^^^^^^^^^^^^
//
// Self-collision checking
// ~~~~~~~~~~~~~~~~~~~~~~~
//
// The first thing we will do is check whether the robot in its
// current state is in *self-collision*, i.e. whether the current
// configuration of the robot would result in the robot's parts
// hitting each other. To do this, we will construct a
// :collision_detection_struct:`CollisionRequest` object and a
// :collision_detection_struct:`CollisionResult` object and pass them
// into the collision checking function. Note that the result of
// whether the robot is in self-collision or not is contained within
// the result. Self collision checking uses an *unpadded* version of
// the robot, i.e. it directly uses the collision meshes provided in
// the URDF with no extra padding added on.

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("1. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");

// Change the state
// ~~~~~~~~~~~~~~~~
//
// Now, let's change the current state of the robot. The planning
// scene maintains the current state internally. We can get a
// reference to it and change it and then check for collisions for the
// new robot configuration. Note in particular that we need to clear
// the collision_result before making a new collision checking
// request.

  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);

  ROS_INFO_STREAM("2. Self collision Test(Change the state): "<< (collision_result.collision ? "in" : "not in"));


// Checking for a group
// ~~~~~~~~~~~~~~~~~~~~
//
// Now, we will do collision checking only for the arm 
// , i.e. we will check whether there are any collisions between
// the  arm and other parts of the body of the robot. We can ask
// for this specifically by adding the group name " arm" to the
// collision request.

  collision_request.group_name = "arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);

  ROS_INFO_STREAM("3. Self collision Test(In a group): "<< (collision_result.collision ? "in" : "not in"));

// Getting Contact Information
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// First, manually set the  arm to a position where we know
// internal (self) collisions do happen. Note that this state is now
// actually outside the joint limits , which we can also
// check for directly.

  std::vector<double> joint_values;
  const robot_model::JointModelGroup* joint_model_group =
    current_state.getJointModelGroup("arm");
  current_state.copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[0] = 1.57; //hard-coded since we know collisions will happen here
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("4. Collision points "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

// Now, we can get contact information for any collisions that might
// have happened at a given configuration of the  arm. We can ask
// for contact information by filling in the appropriate field in the
// collision request and specifying the maximum number of contacts to
// be returned as a large number.

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

//

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);

  ROS_INFO_STREAM("5. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");


  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("6 . Contact between: %s and %s",
             it->first.first.c_str(),
             it->first.second.c_str());
  }

// Modifying the Allowed Collision Matrix
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// The :collision_detection_class:`AllowedCollisionMatrix` (ACM)
// provides a mechanism to tell the collision world to ignore
// collisions between certain object: both parts of the robot and
// objects in the world. We can tell the collision checker to ignore
// all collisions between the links reported above, i.e. even though
// the links are actually in collision, the collision checker will
// ignore those collisions and return not in collision for this
// particular state of the robot.
//
// Note also in this example how we are making copies of both the
// allowed collision matrix and the current state and passing them in
// to the collision checking function.

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for(it2 = collision_result.contacts.begin();
      it2 != collision_result.contacts.end();
      ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

  ROS_INFO_STREAM("6. Self collision Test after modified ACM: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");

  // Full Collision Checking
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // While we have been checking for self-collisions, we can use the
  // checkCollision functions instead which will check for both
  // self-collisions and for collisions with the environment (which is
  // currently empty).  This is the set of collision checking
  // functions that you will use most often in a planner. Note that
  // collision checks with the environment will use the padded version
  // of the robot. Padding helps in keeping the robot further away
  // from obstacles in the environment.*/
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
 
  ROS_INFO_STREAM("6. Full collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " collision");

 
  ros::shutdown();
  return 0;
}
