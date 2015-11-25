#!/usr/bin/env python

'''

launchpad_process.py - Recieve sensor values from launchpad node and process for odometry and speed controller

Created September 2014

Copyright(c) 2014 Lentin Joseph

Some portion borrowed from Rainer Hessmer blog

http://www.hessmer.org/blog/
'''

import rospy
import sys
import time
import math
import tf


from std_msgs.msg import String,Float32, Int32,UInt64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Process_Sensor(object):

	def __init__(self):

		print "Init"
		self._Left_Encoder_Ticks = 0
		self._Right_Encoder_Ticks = 0

		self._Prev_Left_Encoder_Ticks = 0
		self._Prev_Right_Encoder_Ticks = 0


		#Time
		self._last_time_micro_sec = 0
		self._last_time_sec = 0
		
#Subscriber for encoders
		self._Left_Encoder_ = rospy.Subscriber('left_encoder',Float32, self._Update_Left_Encoder)


		self._Right_Encoder_ = rospy.Subscriber('right_encoder',Float32, self._Update_Right_Encoder)

#Subscriber for command velocity

#	        self._cmd_velocity_ = rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand) # Is this line or the below bad redundancy?
		self._cmd_velocity_ = rospy.Subscriber("cmd_vel_mux/input/teleop", Twist, self._HandleVelocityCommand) # IS this line or the above bad redundancy?
		
#Publish speed

		self._left_motor = rospy.Publisher("left_wheel_speed",Int32, queue_size=1)
		self._right_motor = rospy.Publisher("right_wheel_speed",Int32, queue_size=1)


###############################################################
		self._tf_broad_caster = tf.TransformBroadcaster()
		self._odom_publisher = rospy.Publisher("odom",Odometry, queue_size=10)




#Defining robot parameters
#///////////////////////////////////////////////////////////////////////////////////
#Calculating distance per count
#Units are in meters
# Values are hardcorded here itself
#Replacement of Robot params

		self.wheel_diameter = 0.09
		self.trackwidth = 0.01
		self.counts_per_revolution = 4200


		self.distance_per_count = (math.pi * self.wheel_diameter)/ self.counts_per_revolution# //0.00003366
		self.radians_per_count = (self.distance_per_count) / self.trackwidth  #//0.003365992

		self.delta_x = 0
		self.delta_y = 0

		self.X_Pos = 0
		self.Y_Pos = 0

		self.Heading = 0

		self.Velocity = 0
		self.Omega = 0

#////////////////////////////////////////////////////////////////////////////////////
###################################################################
#Subscribe for time


		self._micro_sec_ = rospy.Subscriber('micro_sec', UInt64,self._Update_Micro_Sec)

		self._sec_ = rospy.Subscriber('sec',Float32,self._Update_Sec)


####################################################################3

	def _Update_Left_Encoder(self,left_ticks):
		self._Left_Encoder_Ticks = left_ticks.data
	
	def _Update_Right_Encoder(self,right_ticks):
		self._Right_Encoder_Ticks = right_ticks.data
				
######################################################################

	def _Update_Micro_Sec(self,micro_sec):

		self._last_time_micro_sec = micro_sec.data


	def _Update_Sec(self,sec):
		self._last_time_sec = sec.data

#		rospy.loginfo(self._Left_Encoder_Ticks)
#		rospy.loginfo(self._Right_Encoder_Ticks)
#		rospy.loginfo(self._last_time_sec)
#		rospy.loginfo(self._last_time_micro_sec)

#Update odometry from left and right encoder
		self._Compute_Wheel_Odometry();
#####################################################################3

	def _Compute_Wheel_Odometry(self):

		try:
			delta_Left = self._Left_Encoder_Ticks - self._Prev_Left_Encoder_Ticks
			delta_Right = self._Right_Encoder_Ticks - self._Prev_Right_Encoder_Ticks




			deltaDistance = 0.5 * (delta_Left + delta_Right) * self.distance_per_count

			deltaHeading = (delta_Right - delta_Left) * self.radians_per_count


			self.delta_x = deltaDistance *math.cos(self.Heading)
			self.delta_y = deltaDistance *math.sin(self.Heading)



			self.X_Pos += self.delta_x
			self.Y_Pos += self.delta_y

			self.Heading += deltaHeading

			rospy.loginfo("X_POS")
			rospy.loginfo(self.X_Pos)


			rospy.loginfo("Y_POS")
			rospy.loginfo(self.Y_Pos)

			rospy.loginfo("Heading")
			rospy.loginfo(self.Heading)


			if(self.Heading > math.pi):
				self.Heading -= math.pi * 2
			if(self.Heading <= -math.pi):
				self.Heading += math.pi * 2

			try:
				self.Velocity = deltaDistance / self._last_time_sec
				self.Omega = deltaHeading / self._last_time_sec
			except:
				self.Velocity = 0
				self.Omega = 0
	
			rospy.loginfo("Velocity")
			rospy.loginfo(self.Velocity)


			rospy.loginfo("Omega")
			rospy.loginfo(self.Omega)


			#Call Tf publishing method

			self.Publish_Odom_Tf()

	#		rospy.loginfo(deltaDistance)




			self._Prev_Left_Encoder_Ticks = self._Left_Encoder_Ticks
			self._Prev_Right_Encoder_Ticks = self._Right_Encoder_Ticks
		except:

			rospy.logwarn("Error in odometry computation, robot may be static")
			self.X_Pos = 0
			self.Y_Pos = 0

			self.Heading = 0

			self.Velocity = 0
			self.Omega = 0


			self.Publish_Odom_Tf()



#######################################################################
#Publish Odom and TF
	def Publish_Odom_Tf(self):
		
		
	#quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
		quaternion = Quaternion()
		quaternion.x = 0 
		quaternion.y = 0
		quaternion.z = math.sin(self.Heading / 2.0)
		quaternion.w = math.cos(self.Heading / 2.0)
			
		rosNow = rospy.Time.now()
		self._tf_broad_caster.sendTransform(
				(self.X_Pos, self.Y_Pos, 0), 
					(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
					rosNow,
					"base_footprint",
					"odom"
					)

		# next, we'll publish the odometry message over ROS
		odometry = Odometry()
		odometry.header.frame_id = "odom"
		odometry.header.stamp = rosNow
		odometry.pose.pose.position.x = self.X_Pos
		odometry.pose.pose.position.y = self.Y_Pos
		odometry.pose.pose.position.z = 0
		odometry.pose.pose.orientation = quaternion


		odometry.child_frame_id = "base_link"
		odometry.twist.twist.linear.x = self.Velocity
		odometry.twist.twist.linear.y = 0
		odometry.twist.twist.angular.z = self.Omega
		self._odom_publisher.publish(odometry)
			
#		except:
#			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))
#			rospy.logwarn("Error from odometry pub")

		
		

	def _HandleVelocityCommand(self,twistCommand):

		""" Handle movement requests. """
		v = twistCommand.linear.x        # m/s
		omega = twistCommand.angular.z      # rad/s
		rospy.logwarn("Handling twist command: " + str(v) + "," + str(omega))


#		omega= omega * 1000

		

#		message = 's %.3f %.3f\r' % (v, omega)
		message = 's %.3f %.3f\r' % (v, omega)
#		rospy.logwarn(str(v)+str(omega))
#		rospy.logwarn("Sending speed command message: " + message)

		angularVelocityOffset = 0.5 * omega * self.trackwidth
		speed_limit = 100



                if (v > 0):
                	if ((speed_limit * self.distance_per_count) - math.fabs(angularVelocityOffset) < v):
                	    	v = (speed_limit * self.distance_per_count) - math.fabs(angularVelocityOffset)
                elif(v < 0):
                	if (-((speed_limit * self.distance_per_count) - math.fabs(angularVelocityOffset)) > v):
                	    	v = -((speed_limit * self.distance_per_count) - math.fabs(angularVelocityOffset))

                    
                expectedLeftSpeed = v - angularVelocityOffset
		expectedRightSpeed = v + angularVelocityOffset

		expectedLeftSpeed = expectedLeftSpeed / self.distance_per_count
		expectedRightSpeed = expectedRightSpeed / self.distance_per_count


		NormalizedLeftCV = expectedLeftSpeed
		NormalizedRightCV = expectedRightSpeed
	
		rospy.logwarn("Actual speed")


#		rospy.loginfo(NormalizedLeftCV)
#		rospy.loginfo(NormalizedRightCV)

		old_max = 1
		old_min = -1

		new_max = 1023
		new_min = -1023
		
		old_left_value = NormalizedLeftCV 
		old_right_value = NormalizedRightCV

		
		actual_left_speed = ( (old_left_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min

		actual_right_speed = ( (old_left_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min


		rospy.logwarn("Actual speed")
		rospy.logwarn(actual_left_speed)
		rospy.logwarn(actual_right_speed)

		
#Path for turning
		self.Publish_Speed(actual_left_speed ,actual_right_speed,v,omega)

 
	def Publish_Speed(self,actual_left_speed ,actual_right_speed,v,omega):




		if(v > 0 and omega < 0):		
			self._left_motor.publish(-(actual_left_speed))
			self._right_motor.publish(actual_right_speed)
		elif(v == 0 and omega < 0):
			self._left_motor.publish(actual_left_speed)
			self._right_motor.publish(-(actual_right_speed))
		
		elif(v > 0 and omega > 0):
			self._left_motor.publish(-actual_left_speed)
			self._right_motor.publish(-actual_right_speed)

		elif(v < 0 and omega < 0):
			self._left_motor.publish(actual_left_speed)
			self._right_motor.publish(actual_right_speed)
		elif(v == 0 and omega == 0):
			self._left_motor.publish(0)
			self._right_motor.publish(0)
			

if __name__ =='__main__':

	rospy.init_node('launchpad_process_node',anonymous=True)
	print "Main"
#Create an object of Process_Sensor
	try:	
		sensor_proc = Process_Sensor()
		rospy.spin()

	except rospy.ROSInterruptException:
		print "Test"
	




