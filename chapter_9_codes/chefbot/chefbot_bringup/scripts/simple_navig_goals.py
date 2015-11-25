#!/usr/bin/env python

import rospy
import actionlib

import sys

#move_base_msgs
from move_base_msgs.msg import *

def simple_move(x,w):

    rospy.init_node('simple_move')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
 
    rospy.loginfo("Set X = "+x)
    rospy.loginfo("Set W = "+w)

    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.orientation.w = float(w)
    goal.target_pose.header.frame_id = 'first_move'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    rospy.loginfo("Waiting for server")

    sac.wait_for_server()


    rospy.loginfo("Sending Goals")

    #send goal

    sac.send_goal(goal)
    rospy.loginfo("Waiting for server")

    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()


if __name__ == '__main__':
    try:
        simple_move(sys.argv[1],sys.argv[2])
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
