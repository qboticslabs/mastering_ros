#!/usr/bin/env python
import roslib; roslib.load_manifest('cool5000_controller')
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
import time
import sys
import pickle

joint_names = (
               'joint1_controller',
               'joint2_controller',
               'joint3_controller',
               'joint4_controller',
               'joint5_controller',
               'joint6_controller',
               'joint7_controller'
	)

coords = []   # store an array of coordinates over time

def callback(data):
    #print data
    #rint data.name, data.position
    coords.append(  [data.name, data.current_pos]  )


def listener():
    rospy.init_node('motion_record', anonymous=True)
    for joint_name in joint_names:
        rospy.Subscriber("/"+joint_name+"/state", JointState, callback)

    rospy.spin()

if __name__ == '__main__':

    print 'Disabling Torque to Servos'

    for joint_name in joint_names:
        print 'Looking for service ', joint_name
        rospy.wait_for_service('/'+joint_name+'/torque_enable')

        try:
            torquer = rospy.ServiceProxy('/'+joint_name+'/torque_enable', TorqueEnable)
            response = torquer(0)
            print 'Response from '+joint_name+':', response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    # begin recording joint angles
    
    time.sleep(1)

    print '-----------------------------------------------'
    print 'Listening'
    print '-----------------------------------------------'
    print ''

    listener()
    print 'Done listening'

    # Load the file where we will dump the position data
    f = open('dump.dat','w')
    pickle.dump(coords,f)
    f.close()

    print 'Recording successful'
