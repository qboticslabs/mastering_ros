#!/usr/bin/env python

#Murray demo


import roslib
import rospy

import time
from geometry_msgs.msg import Twist

if __name__== "__main__":

	rospy.init_node('murray_demo')
	pub = rospy.Publisher("cmd_vel_mux/input/teleop",Twist, queue_size=1)
	


while True:
#Go straigt





		for i in range(0,15):

			twist = Twist()
			twist.linear.x = -0.1
			twist.linear.y = 0
			twist.linear.z = 0


			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()

		time.sleep(2)

	#Make a turn

		for i in range(0,1):

		        twist = Twist()
		        twist.linear.x = 0
		        twist.linear.y = 0
		        twist.linear.z = 0


		        twist.angular.x = 0
		        twist.angular.y = 0
		        twist.angular.z = -0.002

		        pub.publish(twist)
		        r = rospy.Rate(1)
		        r.sleep()


		time.sleep(2)
	#Move a little forward


		for i in range(0,1):

			twist = Twist()
			twist.linear.x = -0.001
			twist.linear.y = 0
			twist.linear.z = 0


			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()

		time.sleep(2)


	#Make a turn

		for i in range(0,1):

		        twist = Twist()
		        twist.linear.x = 0
		        twist.linear.y = 0
		        twist.linear.z = 0


		        twist.angular.x = 0
		        twist.angular.y = 0
		        twist.angular.z = -0.002

		        pub.publish(twist)
		        r = rospy.Rate(2)
		        r.sleep()


		time.sleep(2)

	###################################################################################3
'''
	#Go straigt

		for i in range(0,15):

			twist = Twist()
			twist.linear.x = -0.1
			twist.linear.y = 0
			twist.linear.z = 0


			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()

		time.sleep(2)



	#Make a turn

		for i in range(0,1):

		        twist = Twist()
		        twist.linear.x = 0
		        twist.linear.y = 0
		        twist.linear.z = 0


		        twist.angular.x = 0
		        twist.angular.y = 0
		        twist.angular.z = 0.002

		        pub.publish(twist)
		        r = rospy.Rate(10)
		        r.sleep()


		time.sleep(2)
	#Move a little forward


		for i in range(0,1):

			twist = Twist()
			twist.linear.x = -0.001
			twist.linear.y = 0
			twist.linear.z = 0


			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()

		time.sleep(2)


	#Make a turn

		for i in range(0,1):

		        twist = Twist()
		        twist.linear.x = 0
		        twist.linear.y = 0
		        twist.linear.z = 0


		        twist.angular.x = 0
		        twist.angular.y = 0
		        twist.angular.z = 0.002

		        pub.publish(twist)
		        r = rospy.Rate(10)
		        r.sleep()


		time.sleep(4)






		for i in range(0,15):

		        twist = Twist()
		        twist.linear.x = -0.1
		        twist.linear.y = 0
		        twist.linear.z = 0


		        twist.angular.x = 0
		        twist.angular.y = 0
		        twist.angular.z = 0

		        pub.publish(twist)
		        r = rospy.Rate(10)
		        r.sleep()

		time.sleep(2)





        	for i in range(0,8):

                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0


                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = -0.2

                pub.publish(twist)
                r = rospy.Rate(10)
                r.sleep()
'''
