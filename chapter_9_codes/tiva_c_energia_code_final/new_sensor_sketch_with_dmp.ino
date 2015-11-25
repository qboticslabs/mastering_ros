/*
#  Chefbot_ROS_Interface.ino
#  
#  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
#  Website : www.qboticslabs.com , www.lentinjoseph.com
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  Some of the portion is adapted from I2C lib example code for MPU 6050
*/


//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

///////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];


#define OUTPUT_READABLE_QUATERNION
////////////////////////////////////////////////////////////////////////////////////////////////

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
//Encoder pins definition

// Left encoder

#define Left_Encoder_PinA PF_4
#define Left_Encoder_PinB PD_7

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder

#define Right_Encoder_PinA PD_6
#define Right_Encoder_PinB PC_7
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

/////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins

#define A_1 PA_3
#define B_1 PA_4

//PWM 1 pin number
#define PWM_1 PC_6


//Right Motor
#define A_2 PE_4 
#define B_2 PE_5

//PWM 2 pin number
#define PWM_2 PC_5

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition
const int echo = 9, Trig = 10;
long duration, cm;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Battery level monitor for future upgrade
#define BATTERY_SENSE_PIN PC_4

float battery_level = 12;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset

#define RESET_PIN PB_2

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;		
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
/////////////////////////////////////////////////////////////////


//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{
  
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);  
  
  //Setup Encoders
  SetupEncoders();
  //Setup Motors
  SetupMotors();
  //Setup Ultrasonic
  SetupUltrasonic();  
  //Setup MPU 6050
  Setup_MPU6050();
  //Setup Reset pins
  SetupReset();
  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
    
  
  
}

//SetupEncoders() Definition

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);
  

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING); 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{
 
 //Left motor
 pinMode(A_1,OUTPUT);
 pinMode(B_1,OUTPUT); 
 

 //Right Motor
 pinMode(A_2,OUTPUT);
 pinMode(B_2,OUTPUT);  
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup UltrasonicsSensor() function
void SetupUltrasonic()
{
 pinMode(Trig, OUTPUT);
 pinMode(echo, INPUT); 
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function

void Setup_MPU6050()
{


    Wire.begin();
   // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    //Initialize DMP in MPU 6050
    Setup_MPU6050_DMP();
  
 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU 6050 DMP
void Setup_MPU6050_DMP()
{
  
     //DMP Initialization
  
   devStatus = accelgyro.dmpInitialize();
   
   accelgyro.setXGyroOffset(220);
   accelgyro.setXGyroOffset(76);
   accelgyro.setXGyroOffset(-85); 
   accelgyro.setXGyroOffset(1788);  
  
  
   if(devStatus == 0){
    
    accelgyro.setDMPEnabled(true);
    
    pinMode(PUSH2,INPUT_PULLUP);    
    attachInterrupt(PUSH2, dmpDataReady, RISING);
    
    mpuIntStatus = accelgyro.getIntStatus();
    
    dmpReady = true;
    
    packetSize = accelgyro.dmpGetFIFOPacketSize();
     
   }else{
     
     ;
     }
  
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Reset() function

void SetupReset()

{
  
  
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);
  
  
  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  digitalWrite(RESET_PIN,HIGH);
  
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{

    //Read from Serial port
    Read_From_Serial();
    
    
    //Send time information through serial port
    Update_Time();
    
    //Send encoders values through serial port
    Update_Encoders();
    
    //Send ultrasonic values through serial port
    Update_Ultra_Sonic();
        

    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();


    //Send MPU 6050 values through serial port
    Update_MPU6050();
    
    //Send battery values through serial port
    Update_Battery();
    
    
  
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       Messenger_Handler.process(data);
     
     
    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     Set_Speed();
     return; 
    
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
   
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions

void do_Right_Encoder()
{
  
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
 
 
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{
    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
  
  
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reset function
void Reset()
{
 
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW); 
 
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors()
{
  
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);  
  Serial.print("\n");


}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both encoder value through serial port
void Update_Encoders()
{
 
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
  
  
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update ultrasonic sensors through serial port

void Update_Ultra_Sonic()
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  
  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("\n");
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050

void Update_MPU6050()
{
  
  
  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

    ///Update values from DMP for getting rotation vector
    Update_MPU6050_DMP();

 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 DMP functions

void Update_MPU6050_DMP()
{
  
 //DMP Processing

    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {
      
        ;    
      
          }



    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    
    //get current FIFO count
    fifoCount = accelgyro.getFIFOCount();
    
    
    if ((mpuIntStatus & 0x10) || fifoCount > 512) {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
    }




else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

        // read a packet from FIFO
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            
            
            Serial.print("i");Serial.print("\t");
            Serial.print(q.x); Serial.print("\t");
            Serial.print(q.y); Serial.print("\t");
            Serial.print(q.z); Serial.print("\t");
            Serial.print(q.w);
            Serial.print("\n");
    
    
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetAccel(&aa, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            accelgyro.dmpGetQuaternion(&q, fifoBuffer);
            accelgyro.dmpGetAccel(&aa, fifoBuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif


    }
  
   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time()
{
  
      
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
	MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;

    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
 
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update battery function
void Update_Battery()

{
 battery_level = analogRead(PC_4); 
 
 Serial.print("b");
 Serial.print("\t");
 Serial.print(battery_level);
 Serial.print("\n");

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Motor running function


void moveRightMotor(float rightServoValue)
{
  if (rightServoValue>0)
  {
       
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,LOW);
 analogWrite(PWM_1,rightServoValue);
    
  }
  else if(rightServoValue<0)
  {
 digitalWrite(A_1,LOW);
 digitalWrite(B_1,HIGH);
 analogWrite(PWM_1,abs(rightServoValue));
 
  }
  
  else if(rightServoValue == 0)
  {
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,HIGH);
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
 if (leftServoValue > 0)
  {
digitalWrite(A_2,LOW);
digitalWrite(B_2,HIGH);
analogWrite(PWM_2,leftServoValue);
  }
  else if(leftServoValue < 0)
  {
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,LOW);
 analogWrite(PWM_2,abs(leftServoValue));

  }
  else if(leftServoValue == 0)
  {

   digitalWrite(A_2,HIGH);
   digitalWrite(B_2,HIGH);
  
   }  
  
  
}

