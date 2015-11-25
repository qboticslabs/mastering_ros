

#include <ros.h>
#include <std_msgs/String.h>



ros::NodeHandle  nh;

std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

void callback ( const std_msgs::String& msg){

  str_msg.data = msg.data;
  chatter.publish( &str_msg );
    
}

ros::Subscriber<std_msgs::String> sub("talker", callback);


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(3);
}
