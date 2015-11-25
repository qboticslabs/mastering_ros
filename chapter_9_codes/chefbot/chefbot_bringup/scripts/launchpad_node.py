#!/usr/bin/env python

'''
launchpad_node.py - Receive sensor values from Launchpad board and publish as topics

Created September 2014

Copyright(c) 2014 Lentin Joseph

Some portion borrowed from  Rainer Hessmer blog
http://www.hessmer.org/blog/
'''

#Python client library for ROS
import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
#Importing ROS data type for IMU
from sensor_msgs.msg import Imu

#Class to handle serial data from Launchpad and converted to ROS topics
class Launchpad_Class(object):
	
	def __init__(self):
		print "Initializing Launchpad Class"

#######################################################################################################################
		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._battery_value = 0
		self._ultrasonic_value = 0

		self._qx = 0
		self._qy = 0
		self._qz = 0
		self._qw = 0

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0

		self.robot_heading = 0
#######################################################################################################################
		#Get serial port and baud rate of Tiva C Launchpad
		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

#######################################################################################################################
		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")
		

#######################################################################################################################
#Subscribers and Publishers

		#Publisher for left and right wheel encoder values
		self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)		

		#Publisher for Battery level(for upgrade purpose)
		self._Battery_Level = rospy.Publisher('battery_level',Float32,queue_size = 10)
		#Publisher for Ultrasonic distance sensor
		self._Ultrasonic_Value = rospy.Publisher('ultrasonic_distance',Float32,queue_size = 10)

		#Publisher for IMU rotation quaternion values
		self._qx_ = rospy.Publisher('qx',Float32,queue_size = 10)
		self._qy_ = rospy.Publisher('qy',Float32,queue_size = 10)
		self._qz_ = rospy.Publisher('qz',Float32,queue_size = 10)
		self._qw_ = rospy.Publisher('qw',Float32,queue_size = 10)
	
		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		
#######################################################################################################################
#Subscribers and Publishers of IMU data topic

		self.frame_id = '/base_link'

	        self.cal_offset = 0.0
        	self.orientation = 0.0
        	self.cal_buffer =[]
        	self.cal_buffer_length = 1000
        	self.imu_data = Imu(header=rospy.Header(frame_id="base_link"))
        	self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
	        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        	self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        	self.gyro_measurement_range = 150.0 
        	self.gyro_scale_correction = 1.35
        	self.imu_pub = rospy.Publisher('imu/data', Imu,queue_size = 10)

		self.deltat = 0
		self.lastUpdate = 0

#New addon for computing quaternion
		
		self.pi = 3.14159
		self.GyroMeasError = float(self.pi * ( 40 / 180 ))
		self.beta = float(math.sqrt(3 / 4) * self.GyroMeasError)

		self.GyroMeasDrift = float(self.pi * ( 2 / 180 ))
		self.zeta = float(math.sqrt(3 / 4) * self.GyroMeasDrift)


		self.beta = math.sqrt(3 / 4) * self.GyroMeasError

		self.q = [1,0,0,0]
#######################################################################################################################
#Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)

		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)


#######################################################################################################################
	def _Update_Left_Speed(self, left_speed):

		self._left_wheel_speed_ = left_speed.data

		rospy.loginfo(left_speed.data)

		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))

		self._WriteSerial(speed_message)

#######################################################################################################################################################3
				

	def _Update_Right_Speed(self, right_speed):

		self._right_wheel_speed_ = right_speed.data

		rospy.loginfo(right_speed.data)

		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))

		self._WriteSerial(speed_message)


#######################################################################################################################
#Calculate orientation from accelerometer and gyrometer

	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))


		if(len(line) > 0):

			lineParts = line.split('\t')
			try:
				if(lineParts[0] == 'e'):
					self._left_encoder_value = long(lineParts[1])
					self._right_encoder_value = long(lineParts[2])


#######################################################################################################################

					self._Left_Encoder.publish(self._left_encoder_value)
					self._Right_Encoder.publish(self._right_encoder_value)

#######################################################################################################################
				
				if(lineParts[0] == 'b'):
					self._battery_value = float(lineParts[1])

#######################################################################################################################
					self._Battery_Level.publish(self._battery_value)

#######################################################################################################################


				if(lineParts[0] == 'u'):
					self._ultrasonic_value = float(lineParts[1])


#######################################################################################################################
					self._Ultrasonic_Value.publish(self._ultrasonic_value)
#######################################################################################################################
			
				if(lineParts[0] == 'i'):

					self._qx = float(lineParts[1])
					self._qy = float(lineParts[2])
					self._qz = float(lineParts[3])
					self._qw = float(lineParts[4])


#######################################################################################################################
					self._qx_.publish(self._qx)
					self._qy_.publish(self._qy)
					self._qz_.publish(self._qz)
					self._qw_.publish(self._qw)

#######################################################################################################################




					imu_msg = Imu()
					h = Header()
					h.stamp = rospy.Time.now()
					h.frame_id = self.frame_id

					imu_msg.header = h

					imu_msg.orientation_covariance = (-1., )*9	
					imu_msg.angular_velocity_covariance = (-1., )*9
					imu_msg.linear_acceleration_covariance = (-1., )*9


					imu_msg.orientation.x = self._qx
					imu_msg.orientation.y = self._qy
					imu_msg.orientation.z = self._qz
					imu_msg.orientation.w = self._qw

					self.imu_pub.publish(imu_msg)



				
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass
			


#######################################################################################################################


	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

#######################################################################################################################


	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

#######################################################################################################################

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()
		

		
#######################################################################################################################

	
	def Subscribe_Speed(self):
		a = 1
#		print "Subscribe speed"

#######################################################################################################################


	def Reset_Launchpad(self):
		print "Reset"
		reset = 'r\r'
		self._WriteSerial(reset)
		time.sleep(1)
		self._WriteSerial(reset)
		time.sleep(2)


#######################################################################################################################

	def Send_Speed(self):
#		print "Set speed"
		a = 3


if __name__ =='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad = Launchpad_Class()
	try:
		
		launchpad.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")


	launchpad.Reset_Launchpad()
	launchpad.Stop()

#######################################################################################################################


