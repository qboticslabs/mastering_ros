#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include "wiringPi.h"

//Wiring PI 1
#define LED 1

void blink_callback(const std_msgs::Bool::ConstPtr& msg)
{

  
 if(msg->data == 1){

 	digitalWrite (LED, HIGH) ; 
	ROS_INFO("LED ON");


  }

 if(msg->data == 0){

 	digitalWrite (LED, LOW) ; 
	ROS_INFO("LED OFF");

  }

	
}



int main(int argc, char** argv)
{


	ros::init(argc, argv,"blink_led");

	ROS_INFO("Started Odroid-C1 Blink Node");

	wiringPiSetup ();
	pinMode(LED, OUTPUT);


	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("led_blink",10,blink_callback);


	ros::spin();	


	



}

