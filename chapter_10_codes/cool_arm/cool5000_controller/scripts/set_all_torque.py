#!/usr/bin/env python
'''
To set torque enable or disable all at once.

To enable torque
   python set_all_torque_dummy.py 1
To desable torque
   python set_all_torque_dummy.py 0

If the packets are missing in some servos in the first run , Then run it multiple times

It uses the service /'joint_name'/torque_enable corresponding to individual dynamixels.
'''

import roslib
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
import time
import sys

joint_names = (
               'joint1_controller',
               'joint2_controller',
               'joint3_controller',
               'joint4_controller',
               'joint5_controller',
               'joint6_controller',
               'joint7_controller'
		)

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print 'Usage: set_all_torque.py <0,1>\n'
        print 'Controls if the motors are activated and have torque\n'
        print '1 is True, 0 is False\n'
        sys.exit(1)

    print 'Setting torque to '+sys.argv[1]

    ivalue  = int(sys.argv[1])

    for joint_name in joint_names:
        print 'Looking for service ', joint_name
        rospy.wait_for_service('/'+joint_name+'/torque_enable')

        try:
            torquer = rospy.ServiceProxy('/'+joint_name+'/torque_enable', TorqueEnable)
            response = torquer(ivalue)
            print 'Response from '+joint_name+':', response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

