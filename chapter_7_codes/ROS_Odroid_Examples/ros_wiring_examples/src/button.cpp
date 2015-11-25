#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include "wiringPi.h"

//Wiring PI 1
#define BUTTON 0
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

	ros::init(argc, argv,"button_led");
	ROS_INFO("Started Odroid-C1 Button Blink Node");


	wiringPiSetup ();

	pinMode(LED, OUTPUT);
	pinMode(BUTTON, INPUT);
    	pullUpDnControl(BUTTON, PUD_UP); // Enable pull-up resistor on button


	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber sub = n.subscribe("led_blink",10,blink_callback);
  	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("led_blink", 10);


	std_msgs::Bool button_press;
	button_press.data = 1;

	std_msgs::Bool button_release;
	button_release.data = 0;

 	while (ros::ok())
  	{

	        if (!digitalRead(BUTTON)) // Button is released if this returns 1
		{
	
			ROS_INFO("Button Pressed");
			chatter_pub.publish(button_press);

		}
		else
		{

			ROS_INFO("Button Released");
			chatter_pub.publish(button_release);

		}

		ros::spinOnce();
    		loop_rate.sleep();

	}

	



}

