#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include "wiringPi.h"

//Wiring PI 1
#define BUTTON 0
#define LED 1

ros::Rate loop_rate(10);

void blink_callback(const std_msgs::Bool::ConstPtr& msg)
{

  ROS_INFO("I heard: [%d]", msg->data);
  
 if(msg->data == 1){

 	digitalWrite (LED, HIGH) ; 
  }

 if(msg->data == 0){

 	digitalWrite (LED, LOW) ; 
  }

	
}



int main(int argc, char** argv)
{


	ros::init(argc, argv,"button_led");

	ROS_INFO("Started Odroid-C1 Button Blink Node");

	wiringPiSetup ();
	pinMode(LED, OUTPUT);


	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("led_blink",10,blink_callback);
  	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("led_blink", 10);


	std_msgs::Bool on = true;
	std_msgs::Bool off = false;


 	while (ros::ok())
  	{

	        if (digitalRead(butPin)) // Button is released if this returns 1
		{
	
			chatter_pub.publish(on);

		}
		else
		{

			chatter_pub.publish(off);

		}

		ros::spinOnce();

    		loop_rate.sleep();

	}

	



}

