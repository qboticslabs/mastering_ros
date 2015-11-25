#!/usr/bin/env python

"""

    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
import sys

#import roslib

#; roslib.load_manifest('differential_drive')
import rospy
from geometry_msgs.msg import Twist
import inspect, os


from PySide import QtGui, QtCore

##########################################################################
##########################################################################
class MainWindow(QtGui.QMainWindow):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self):
    #####################################################################    
        super(MainWindow, self).__init__()
        self.timer_rate = rospy.get_param('~publish_rate', 50)
        self.pub_twist = rospy.Publisher('twist', Twist,queue_size=10)
        
        self.initUI()
        
    #####################################################################    
    def initUI(self):      
    #####################################################################    
        
        img_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/../images/crosshair.jpg"
        rospy.loginfo("initUI img_path: %s" % img_path)

        self.statusBar()
        
        self.setStyleSheet("QMainWindow { border-image: url(%s); }" % img_path)
        
                
        self.setGeometry(0, 600, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()
        
        self.statusBar().showMessage('started')
        
    #####################################################################    
    def mousePressEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        self.get_position(event)
        
    #####################################################################    
    def mouseReleaseEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse released')
        self.timer.stop()
        
    #####################################################################    
    def mouseMoveEvent(self, event):
    #####################################################################    
        self.get_position(event)
        
    #####################################################################    
    def get_position(self, event):
    #####################################################################    
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
        self.statusBar().showMessage('point (%0.2f, %0.2f)' % (self.x,self.y))
        
    #####################################################################    
    def timerEvent(self, event):
    #####################################################################    
        # self.statusBar().showMessage("timer tick")
        self.pubTwist()
        
    #######################################################
    def pubTwist(self):
    #######################################################
        # rospy.loginfo("publishing twist from (%0.3f,%0.3f)" %(self.x,self.y))
        self.twist = Twist()
        self.twist.linear.x = (1-self.y) * (x_max - x_min) + x_min
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = (1-self.x) * (r_max - r_min) + r_min
        
        if self.twist.linear.x > x_max:
            self.twist.linear.x = x_max
        if self.twist.linear.x < x_min:
            self.twist.linear.x = x_min
        if self.twist.angular.z > r_max:
            self.twist.angular.z = r_max
        if self.twist.angular.z < r_min:
            self.twist.angular.z = r_min
        
        self.pub_twist.publish( self.twist )
        
##########################################################################
##########################################################################
def main():
##########################################################################
##########################################################################
    rospy.init_node('virtual_joystick')
    rospy.loginfo('virtual_joystick started')
    global x_min
    global x_max
    global r_min
    global r_max
    
    x_min = rospy.get_param("~x_min", -0.20)
    x_max = rospy.get_param("~x_max", 0.20)
    r_min = rospy.get_param("~r_min", -1.0)
    r_max = rospy.get_param("~r_max", 1.0)
    
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
