#!/usr/bin/env python
        
import roslib; roslib.load_manifest('cool5000_controller')
import time
import rospy
import pickle
import traceback
import sys
from std_msgs.msg import Float64

joint_names = ('joint1',
               'joint2',
               'joint3',
               'joint4',
               'joint5',
               'joint6',
               'joint7')


controller_names = ('joint1_controller',
                    'joint2_controller',
                    'joint3_controller',
                    'joint4_controller',
                    'joint5_controller',
                    'joint6_controller',
                    'joint7_controller')
               
def main():

    pubs = [rospy.Publisher(name + '/command', Float64) for name in controller_names]
    rospy.init_node('motion_play', anonymous=True)
    
    # determine file name to open
    if len(sys.argv) != 2:
        filename = 'dump.dat'
    else:
        filename = sys.argv[1]

    f = open(filename,'r')
    read_coords = pickle.load(f)

    print 'Playing...'

    for coor in read_coords:
        print 'sending command', coor[1], "to", coor[0], "via", joint_names.index(coor[0])
        pubs[  joint_names.index(coor[0])  ].publish(coor[1])

        time.sleep(.5)

    print 'COMPLETE ---------------------------- '


# Where the program starts                                                                                          
if __name__ == "__main__":
    try:
        main()
    # Now attempt to prevent the robot from falling limp                                                            
    except KeyboardInterrupt:
        print "done"
    except Exception as e:
        traceback.print_exc(e)
        print "done"
        
        
        

