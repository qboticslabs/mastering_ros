/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.

 * Email id : qboticslabs@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.


* This code will publish a integers from 0 to n with a delay of 100ms

 */

//roscpp Core header 
#include "ros/ros.h"
//Header of Int32 standard message
#include "std_msgs/Int32.h"
#include <iostream>


int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_publisher
	ros::init(argc, argv,"demo_topic_publisher");

	//Created a node handle object
	ros::NodeHandle node_obj;

	//Created a publisher object
	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",10);

	//Created a rate object
	ros::Rate loop_rate(10);

	//Variable of the number initializing as zero
	int number_count = 0;

	//While loop for incrementing number and publishing to topic /numbers
	while (ros::ok())
	{

		//Created a Int32 message
		std_msgs::Int32 msg;

		//Inserted data to message header
		msg.data = number_count;

		//Printing message data
		ROS_INFO("%d",msg.data);

		//Publishing the topic 
		number_publisher.publish(msg);

		//Spinning once for doing the  all operation once
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//Incrementing the count
		++number_count;
	}
	
	return 0;
}


