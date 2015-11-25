

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>



const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)


ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;

ros::Publisher pub("adc", &adc_msg);

void setup()
{

  nh.initNode();

  nh.advertise(pub);

}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}


void loop()
{

  adc_msg.adc0 = averageAnalog(xpin);
  adc_msg.adc1 = averageAnalog(ypin);
  adc_msg.adc2 = averageAnalog(zpin);

  
  pub.publish(&adc_msg);

  nh.spinOnce();

  delay(10);

}
