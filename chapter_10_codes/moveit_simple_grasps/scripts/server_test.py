#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, PAL Robotics SL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the PAL Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Bence Magyar

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import Header
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal
from moveit_msgs.msg import Grasp
import geometry_msgs

grasp_publisher = None
grasps_ac = None

def generate_grasps(pose, width):
	#send request to block grasp generator service
	grasps_ac.wait_for_server()
	rospy.loginfo("Successfully connected.")
	goal = GenerateGraspsGoal()
	goal.pose = pose.pose
	goal.width = width
	grasps_ac.send_goal(goal)
	rospy.loginfo("Sent goal, waiting:\n" + str(goal))
	t_start = rospy.Time.now()
	grasps_ac.wait_for_result()
	t_end = rospy.Time.now()
	t_total = t_end - t_start
	rospy.loginfo("Result received in " + str(t_total.to_sec()))
	grasp_list = grasps_ac.get_result().grasps
	return grasp_list

def publish_grasps_as_poses(grasps):
	rospy.loginfo("Publishing PoseArray on /grasp_pose_from_block_bla for grasp_pose")
	graspmsg = Grasp()
	grasp_PA = PoseArray()
	header = Header()
	header.frame_id = "base_link"
	header.stamp = rospy.Time.now()
	grasp_PA.header = header
	for graspmsg in grasps:
		p = Pose(graspmsg.grasp_pose.pose.position, graspmsg.grasp_pose.pose.orientation)
		grasp_PA.poses.append(p)
	grasp_publisher.publish(grasp_PA)
	rospy.loginfo('Published ' + str(len(grasp_PA.poses)) + ' poses')
	rospy.sleep(2)


if __name__ == '__main__':
	name = 'grasp_object_server'
	rospy.init_node(name, anonymous=False)
	rospy.loginfo("Connecting to grasp generator AS")
	grasps_ac = SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
	grasp_publisher = rospy.Publisher("generated_grasps", PoseArray)
	object_pose = geometry_msgs.msg.PoseStamped()
	object_pose.pose.position.x = 1.0
	object_pose.pose.position.y = 1.0
	object_pose.pose.position.z = 1.0
	object_pose.pose.orientation.w = 1.0
	object_pose.pose.orientation.x = 0.0
	object_pose.pose.orientation.y = 0.0
	object_pose.pose.orientation.z = 0.0
	grasp_list = generate_grasps(object_pose, 0.06)
	rospy.loginfo('Generated ' + str(len(grasp_list)) + ' grasps.')
	publish_grasps_as_poses(grasp_list)
	rospy.sleep(10.0)
