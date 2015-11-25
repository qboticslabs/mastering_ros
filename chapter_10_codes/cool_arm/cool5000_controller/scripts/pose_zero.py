#!/usr/bin/env python

import roslib
import time
import rospy
from std_msgs.msg import Float64

joint_names = ( 'joint1_controller',
				'joint2_controller',		
				'joint3_controller',
				'joint4_controller',
				'joint5_controller',
				'joint6_controller',
				'joint7_controller')
               
joint_commands = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_zero_pose', anonymous=True)
    
    for i in range(len(pubs)):
        time.sleep(.2)
        print "Sending command"
        pubs[i].publish(joint_commands[i])
