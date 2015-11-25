#!/usr/bin/env python
'''
Created February, 2011

@author: Dr. Rainer Hessmer

  DeadReckoning.py - A Python implementation of the tutorial
  http://www.ros.org/wiki/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.

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
from tf import transformations
import time
import math
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#from velocityLogger import VelocityLogger

class Driver(object):
	'''
	Implements the logic for driving a given distance or turning for a given amount
	by monitoring the transform messages that contain the odometry based pose.
	'''

	def __init__(self):
		rospy.init_node('DeadReckoning')
		
		self._VelocityCommandPublisher = rospy.Publisher("cmd_vel", Twist)
		self._TransformListener = tf.TransformListener()

		# wait for the listener to get the first transform message
		self._TransformListener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(4.0))

	def driveX(self, distance, speed):
		'''
		Drive in x direction a specified distance based on odometry information
		distance [m]: the distance to travel in the x direction (>0: forward, <0: backwards)
		speed [m/s]: the speed with which to travel; must be positive
		'''

		forward = (distance >= 0)
		
		# record the starting transform from the odom to the base_link frame
		# Note that here the 'from' frame precedes 'to' frame which is opposite to how they are
		# ordered in tf.TransformBroadcaster's sendTransform function.
		# startTranslation is a tuple holding the x,y,z components of the translation vector
		# startRotation is a tuple holding the four components of the quaternion
		(startTranslation, startRotation) = self._TransformListener.lookupTransform("/odom", "/base_link", rospy.Time(0))
		
		done = False

		velocityCommand = Twist()
		if forward:
			velocityCommand.linear.x = speed # going forward m/s
		else:
			velocityCommand.linear.x = -speed # going forward m/s
			
		velocityCommand.angular.z = 0.0 # no angular velocity

		while not rospy.is_shutdown():
			try:
				(currentTranslation, currentRotation) = self._TransformListener.lookupTransform("/odom", "/base_link", rospy.Time(0))
				

				print "Current Tr",currentTranslation
				print "CUrrent Rot",currentRotation

				dx = currentTranslation[0] - startTranslation[0]

				dy = currentTranslation[1] - startTranslation[1]
				

				print "Dx",dx
				print "Dy",dy
				distanceMoved = math.sqrt(dx * dx + dy * dy)


				print "Distance moved",distanceMoved

				if (forward):
					arrived = distanceMoved >= distance
				else:
					arrived = distanceMoved >= -distance
				
				if (arrived):
					break
				else:
					# send the drive command
					print("sending vel command" + str(velocityCommand))
					self._VelocityCommandPublisher.publish(velocityCommand)
				
			except (tf.LookupException, tf.ConnectivityException):
				continue

			rospy.sleep(0.1)

		#stop
		velocityCommand.linear.x = 0.0
		velocityCommand.angular.z = 0.0
		self._VelocityCommandPublisher.publish(velocityCommand)
		
		return done


	def turn(self, angle, angularSpeed):
		'''
		Turn the robot based on odometry information
		angle [rad]: the angle to turn (positive angles mean clockwise rotation)
		angularSpeed [rad/s]: the speed with which to turn; must be positive
		'''

		ccw = (angle >= 0) # counter clockwise rotation
		
		# record the starting transform from the odom to the base frame
		# Note that here the 'from' frame precedes 'to' frame which is opposite to how they are
		# ordered in tf.TransformBroadcaster's sendTransform function.
		(startTranslation, startRotation) = self._TransformListener.lookupTransform("/odom", "/base_link", rospy.Time(0))
		startAngle = 2 * math.atan2(startRotation[2], startRotation[3])

		print "start angle: " + str(startAngle)
		previousAngle = startAngle
		angleOffset = 0.0
		
		done = False

		velocityCommand = Twist()
		velocityCommand.linear.x = 0.0 # going forward m/s
		if ccw:
			velocityCommand.angular.z = angularSpeed
		else:
			velocityCommand.angular.z = -angularSpeed

		while not rospy.is_shutdown():
			try:
				(currentTranslation, currentRotation) = self._TransformListener.lookupTransform("/odom", "/base_link", rospy.Time(0))
				currentAngle = 2 * math.atan2(currentRotation[2], currentRotation[3])
				print "currentAngle: " + str(currentAngle)
				
				# we need to handle roll over of the angle
				if (currentAngle * previousAngle < 0 and math.fabs(currentAngle - previousAngle) > math.pi / 2):
					if (currentAngle > previousAngle):
						print "subtracting"
						angleOffset = angleOffset - 2 * math.pi
					else:
						print "adding"
						angleOffset = angleOffset + 2 * math.pi
				
				angleTurned = currentAngle + angleOffset - startAngle
				previousAngle = currentAngle
				
				print "angleTurned: " + str(angleTurned)
				if (ccw):
					arrived = (angleTurned >= angle)
				else:
					arrived = (angleTurned <= angle)
				
				print arrived

				if (arrived):
					break
				else:
					# send the drive command
					print("sending vel command" + str(velocityCommand))
					self._VelocityCommandPublisher.publish(velocityCommand)
				
			except (tf.LookupException, tf.ConnectivityException):
				continue

			time.sleep(0.1)

		#stop
		velocityCommand.linear.x = 0.0
		velocityCommand.angular.z = 0.0
		self._VelocityCommandPublisher.publish(velocityCommand)
		
		return done

if __name__ == '__main__':
	velocityRecordingFilePath = "./RecordedVelocity.txt"
#	if (len(sys.argv) > 1):
		# we accept the path to the goals text file as a command line argument
#		velocityRecordingFilePath = sys.argv[1]

#	velocityLogger = VelocityLogger(velocityRecordingFilePath)
	try:
		driver = Driver()
#		velocityLogger.start()
		print "M1"
		driver.driveX(distance = 0.6, speed = 0.3)
		print "M2"

#		driver.turn(angle =  math.pi, angularSpeed = 60)
#		print "M3"

		driver.driveX(distance = 0, speed = 5)
#		print "M4"

#		driver.turn(angle = -math.pi, angularSpeed = 0.3)
#		print "M5"

#		driver.turn(angle = 3 * math.pi, angularSpeed = 0.3)
#		print "M6"

	except rospy.ROSInterruptException:
		print "Error from main loop"
		pass
	finally:
		pass
#		velocityLogger.close()

