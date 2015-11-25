#!/usr/bin/env python

'''
launchpad_node.py - Recieve sensor values from Launchpad board and publish as topics

Created September 2014

Copyright(c) 2014 Lentin Joseph

Some portion borrowed from  Rainer Hessmer blog

http://www.hessmer.org/blog/
'''

import rospy
import sys
import time

from SerialDataGateway import SerialDataGateway

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt64


class Launchpad_Class(object):
	'''
	Recieve serial data and Publish serial data into 		topics		

	'''





	def __init__(self):
		print "Init"

#######################################################################################################################
#Sensor variables

		self._Counter = 0
		self._left_encoder_value = 0
		self._right_encoder_value = 0

		self._battery_value = 0
		self._ultrasonic_value = 0

		self._ax = 0
		self._ay = 0
		self._az = 0

		self._gx = 0
		self._gy = 0
		self._gz = 0


		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0


		self.robot_heading = 0
#######################################################################################################################
		
		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

#######################################################################################################################
		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		print "Started serial"
		

#######################################################################################################################
#Subscribers and Publishers

		self._Left_Encoder = rospy.Publisher('left_encoder',Float32,queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('right_encoder',Float32,queue_size = 10)		
		self._Battery_Level = rospy.Publisher('battery_level',Float32,queue_size = 10)
		self._Ultrasonic_Value = rospy.Publisher('ultrasonic_distance',Float32,queue_size = 10)

		self._ax_ = rospy.Publisher('ax',Float32,queue_size = 10)
		self._ay_ = rospy.Publisher('ay',Float32,queue_size = 10)
		self._az_ = rospy.Publisher('az',Float32,queue_size = 10)


		self._gx_ = rospy.Publisher('gx',Float32,queue_size = 10)
		self._gy_ = rospy.Publisher('gy',Float32,queue_size = 10)
		self._gz_ = rospy.Publisher('gz',Float32,queue_size = 10)

		self._micro_sec_ = rospy.Publisher('micro_sec',UInt64,queue_size = 10)
		self._sec_ = rospy.Publisher('sec',Float32,queue_size = 10)


#######################################################################################################################
#Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Int32,self._Update_Left_Speed)

		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Int32,self._Update_Right_Speed)


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

	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		#rospy.logdebug(str(self._Counter) + " " + line)
		#if (self._Counter % 50 == 0):
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
#		rospy.loginfo(line)

		if(len(line) > 0):

			lineParts = line.split('\t')

			
			try:
				if(lineParts[0] == 'e'):

					self._left_encoder_value = float(lineParts[1])
					self._right_encoder_value = float(lineParts[2])


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

					self._ax = float(lineParts[1])
					self._ay = float(lineParts[2])
					self._az = float(lineParts[3])


#######################################################################################################################
					self._ax_.publish(self._ax)
					self._ay_.publish(self._ay)
					self._az_.publish(self._az)

#######################################################################################################################


					self._gx = float(lineParts[4])
					self._gy = float(lineParts[5])
					self._gz = float(lineParts[6])



#######################################################################################################################

					self._gx_.publish(self._gx)
					self._gy_.publish(self._gy)
					self._gz_.publish(self._gz)

#######################################################################################################################


				if(lineParts[0] == 't'):
			
					self._LastUpdate_Microsec = long(lineParts[1])
					self._Second_Since_Last_Update = float(lineParts[2])
					
					

					self._micro_sec_.publish(self._LastUpdate_Microsec)
					self._sec_.publish(self._Second_Since_Last_Update)

					


				
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
	
	r = rospy.Rate(10)

	launchpad = Launchpad_Class()
	try:
		
		launchpad.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Test"


	launchpad.Reset_Launchpad()
	launchpad.Stop()

#######################################################################################################################


'''
	while not rospy.is_shutdown():
		try:
			
			launchpad.Send_Speed()
#			launchpad.Read_From_Serial()
#			launchpad.Publish_Sensor_Values()
					
		except rospy.ROSInterruptException: 

			print "Error"
#			launchpad.Stop()
			pass

	launchpad.Reset_Launchpad()
#rospy.ROSInterruptException: pass

'''	

