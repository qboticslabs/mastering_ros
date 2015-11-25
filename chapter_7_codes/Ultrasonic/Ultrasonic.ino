

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;


#define echoPin 7 // Echo Pin
#define trigPin 8 // Trigger Pin

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);


char frameid[] = "/ultrasound";


void setup() {
 
  nh.initNode();
  nh.advertise(pub_range);
  
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  
  
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 60;
  

 
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
  
}

float getRange_Ultrasound(){

  int val = 0;

 for(int i=0; i<4; i++) { 
  
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
  val += duration;
 
 }
 return val / 232.8 ;
  
}
long range_time;

void loop() {
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 

 

   if ( millis() >= range_time ){
    int r =0;


    range_msg.range = getRange_Ultrasound();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  
  nh.spinOnce();
 

 delay(50);
}
