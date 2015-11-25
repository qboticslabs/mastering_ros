#!/usr/bin/env python
'''
Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib; roslib.load_manifest('ardros')
import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ardros.srv import *
#from ardros.msg import *

from SerialDataGateway import SerialDataGateway

class Arduino(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

	CONTROLLER_RESET_REQUIRED = 0;
	CONTROLLER_INITIALIZING = 1;
	CONTROLLER_IS_READY = 2;

	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		#rospy.logdebug(str(self._Counter) + " " + line)
		#if (self._Counter % 50 == 0):
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

		if (len(line) > 0):
			lineParts = line.split('\t')
			self._State = Arduino.CONTROLLER_INITIALIZING

#			if (self._State == Arduino.CONTROLLER_RESET_REQUIRED):
#				if (lineParts[0] == "reset_done"):
#					return
#				else:
#					self._WriteSerial('reset\r')
#					return

			if (self._State == Arduino.CONTROLLER_INITIALIZING):
				if (lineParts[0] == "initialized"):
					self._State = Arduino.CONTROLLER_IS_READY
					return
				elif (lineParts[0] == "InitializeDriveGeometry"):
					# controller requesting initialization
					self._InitializeDriveGeometry()
					return
				elif (lineParts[0] == "InitializeSpeedController"):
					# controller requesting initialization
					self._InitializeSpeedController()
					return
				elif (lineParts[0] == "InitializeBatteryMonitor"):
					# controller requesting initialization
					self._InitializeBatteryMonitor()
					return
					

			if (lineParts[0] == 'o'):
				self._BroadcastOdometryInfo(lineParts)
				return
			if (lineParts[0] == 'b'):
				self._BroadcastBatteryInfo(lineParts)
				return

	def _BroadcastOdometryInfo(self, lineParts):
		partsCount = len(lineParts)
		#rospy.logwarn(partsCount)
		if (partsCount  < 6):
			pass
		
		try:
			x = int(lineParts[1]) / 1000.0
			y = int(lineParts[2]) / 1000.0
			theta = int(lineParts[3]) / 1000.0
			
			vx = int(lineParts[4]) / 1000.0
			omega = int(lineParts[5]) / 1000.0
		
			#quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
			quaternion = Quaternion()
			quaternion.x = 0.0 
			quaternion.y = 0.0
			quaternion.z = sin(theta / 2.0)
			quaternion.w = cos(theta / 2.0)
			
			
			rosNow = rospy.Time.now()
			
			# First, we'll publish the transform from frame odom to frame base_link over tf
			# Note that sendTransform requires that 'to' is passed in before 'from' while
			# the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
			self._OdometryTransformBroadcaster.sendTransform(
				(x, y, 0), 
				(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
				rosNow,
				"base_link",
				"odom"
				)

			# next, we'll publish the odometry message over ROS
			odometry = Odometry()
			odometry.header.frame_id = "odom"
			odometry.header.stamp = rosNow
			odometry.pose.pose.position.x = x
			odometry.pose.pose.position.y = y
			odometry.pose.pose.position.z = 0
			odometry.pose.pose.orientation = quaternion

			odometry.child_frame_id = "base_link"
			odometry.twist.twist.linear.x = vx
			odometry.twist.twist.linear.y = 0
			odometry.twist.twist.angular.z = omega

			self._OdometryPublisher.publish(odometry)
			
			#rospy.loginfo(odometry)
		
		except:
			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))


	def _BroadcastBatteryInfo(self, lineParts):
		partsCount = len(lineParts)
		#rospy.logwarn(partsCount)

		if (partsCount  < 1):
			pass
		
		try:
			batteryVoltage = float(lineParts[1])
			batteryState = BatteryState()
			batteryState.voltage = batteryVoltage
			
			if (batteryVoltage <= self._VoltageLowlimit):
				batteryState.isLow = 1
			if (batteryVoltage <= self._VoltageLowLowlimit):
				batteryState.isLowLow = 1;

#			self._BatteryStatePublisher.publish(batteryState)
			
			#rospy.loginfo(batteryState)
		
		except:
			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

	def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		self._Counter = 0

		rospy.init_node('arduino')

		port = rospy.get_param("~port", "/dev/ttyUSB0")
		baudRate = int(rospy.get_param("~baudRate", 9600))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
		self._SerialPublisher = rospy.Publisher('serial', String)

		self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
		self._OdometryPublisher = rospy.Publisher("odom", Odometry)

#		self._VoltageLowlimit = rospy.get_param("~batteryStateParams/voltageLowlimit", "12.0")
#		self._VoltageLowLowlimit = rospy.get_param("~batteryStateParams/voltageLowLowlimit", "11.7")

#		self._BatteryStatePublisher = rospy.Publisher("battery", BatteryState)
		
		self._SetDriveGainsService = rospy.Service('setDriveControlGains', SetDriveControlGains, self._HandleSetDriveGains)

		self._State = Arduino.CONTROLLER_RESET_REQUIRED

		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()
		
	def _HandleVelocityCommand(self, twistCommand):
		""" Handle movement requests. """
		v = twistCommand.linear.x        # m/s
		omega = twistCommand.angular.z      # rad/s
		rospy.logwarn("Handling twist command: " + str(v) + "," + str(omega))

		message = 's %.2f %.2f\r' % (v, omega)
		rospy.logdebug("Sending speed command message: " + message)
		self._WriteSerial(message)

	def _InitializeDriveGeometry(self):
		wheelDiameter = rospy.get_param("~driveGeometry/wheelDiameter", "0")
		trackWidth = rospy.get_param("~driveGeometry/trackWidth", "0")
		countsPerRevolution = rospy.get_param("~driveGeometry/countsPerRevolution", "0")

		#wheelDiameterParts = self._GetBaseAndExponent(wheelDiameter)
		#trackWidthParts = self._GetBaseAndExponent(trackWidth)

		message = 'dg %f %f %d\r' % (wheelDiameter, trackWidth, countsPerRevolution)
		rospy.logdebug("Sending drive geometry params message: " + message)
		self._WriteSerial(message)

	def _InitializeSpeedController(self):
		velocityPParam = rospy.get_param("~speedController/velocityPParam", "0")
		velocityIParam = rospy.get_param("~speedController/velocityIParam", "0")
		turnPParam = rospy.get_param("~speedController/turnPParam", "0")
		turnIParam = rospy.get_param("~speedController/turnIParam", "0")
		commandTimeout = self._GetCommandTimeoutForSpeedController()

		message = 'sc %.2f %.2f %.2f %.2f %.2f\r' % (velocityPParam, velocityIParam, turnPParam, turnIParam, commandTimeout)
		#message = 'sc %f %f %f\r' % (velocityPParam, velocityIParam, turnPParam)
		rospy.logdebug("Sending differential drive gains message: " + message)
		self._WriteSerial(message)


		#speedControllerParams = (velocityPParam, velocityIParam, turnPParam, turnIParam, commandTimeout)
		#rospy.loginfo(str(speedControllerParams))
		#self._WriteSpeedControllerParams(speedControllerParams)
	
	def _GetCommandTimeoutForSpeedController(self):
		"""
		Returns the command timeout for the speed controller in seconds.
		If no velocity command arrives for more than the specified timeout then the speed controller will stop the robot.
		"""
		return rospy.get_param("~speedController/commandTimeout", "0.5")

	def _HandleSetDriveGains(self, request):
		""" Handle the setting of the drive gains (PID). """
		
		# We persist the new values in the parameter server
		rospy.set_param("~speedController", {'velocityPParam': request.velocityPParam, 'velocityPParam': request.velocityIParam, 'turnPParam': request.turnPParam, 'turnIParam': request.turnIParam})
		
		commandTimeout = self._GetCommandTimeoutForSpeedController()
		speedControllerParams = (request.velocityPParam, request.velocityIParam, request.turnPParam, request.turnIParam, commandTimeout)
		self._WriteSpeedControllerParams(speedControllerParams)
		return SetDriveControlGainsResponse()

	def _WriteSpeedControllerParams(self, speedControllerParams):
		""" Writes the speed controller parameters (drive gains (PID), and command timeout) to the Arduino controller. """
		rospy.logdebug("Handling '_WriteSpeedControllerParams'; received parameters " + str(speedControllerParams))
		
		message = 'SpeedCo %d %d %d %d %d %d %d %d %d %d\r' % self._GetBaseAndExponents(speedControllerParams)
		message = 'SpeedCo 763 -4 3700 -4 9750\r'
		rospy.logdebug("Sending differential drive gains message: " + message)
		self._WriteSerial(message)

	def _InitializeBatteryMonitor(self):
		rospy.logdebug("Initializing battery monitor. voltageTooLowLimit = " + str(self._VoltageLowLowlimit))

		message = 'bm %f\r' % self._VoltageLowLowlimit
		rospy.logdebug("Sending battery monitor params message: " + message)
		self._WriteSerial(message)

	def _GetBaseAndExponent(self, floatValue, resolution=4):
		'''
		Converts a float into a tuple holding two integers:
		The base, an integer with the number of digits equaling resolution.
		The exponent indicating what the base needs to multiplied with to get
		back the original float value with the specified resolution. 
		'''

		if (floatValue == 0.0):
			return (0, 0)
		else:
			exponent = int(1.0 + math.log10(abs(floatValue)))
			multiplier = math.pow(10, resolution - exponent)
			base = int(floatValue * multiplier)

			return(base, exponent - resolution)

	def _GetBaseAndExponents(self, floatValues, resolution=4):
		'''
		Converts a list or tuple of floats into a tuple holding two integers for each float:
		The base, an integer with the number of digits equaling resolution.
		The exponent indicating what the base needs to multiplied with to get
		back the original float value with the specified resolution. 
		'''

		baseAndExponents = []
		for floatValue in floatValues:
			baseAndExponent = self._GetBaseAndExponent(floatValue)
			baseAndExponents.append(baseAndExponent[0])
			baseAndExponents.append(baseAndExponent[1])

		return tuple(baseAndExponents)


if __name__ == '__main__':
	arduino = Arduino()
	try:
		arduino.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		arduino.Stop()


