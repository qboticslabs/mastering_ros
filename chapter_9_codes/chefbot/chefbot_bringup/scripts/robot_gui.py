#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot.ui'
#
# Created: Sat Feb 21 20:25:38 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!


import rospy
import actionlib
from move_base_msgs.msg import *
import time


from PyQt4 import QtCore, QtGui


table_position = dict()
table_position[0] = (-0.465, 0.37, 0.010, 0, 0, 0.998, 0.069)
table_position[1] = (0.599, 1.03, 0.010, 0, 0, 1.00, -0.020)
table_position[2] = (4.415, 0.645, 0.010, 0, 0, -0.034, 0.999)
table_position[3] = (7.409, 0.812, 0.010, 0, 0, -0.119, 0.993)
table_position[4] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[5] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[6] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[7] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[8] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[9] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)




try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(376, 338)
        self.spinBox = QtGui.QSpinBox(Form)
        self.spinBox.setGeometry(QtCore.QRect(20, 160, 161, 121))
        font = QtGui.QFont()
        font.setPointSize(35)
        font.setBold(True)
        font.setWeight(75)
        self.spinBox.setFont(font)
        self.spinBox.setMaximum(9)
        self.spinBox.setObjectName(_fromUtf8("spinBox"))
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(20, 120, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.pushButton = QtGui.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(220, 190, 131, 41))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_2 = QtGui.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(220, 240, 131, 41))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.pushButton_3 = QtGui.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(220, 140, 131, 41))
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.progressBar = QtGui.QProgressBar(Form)
        self.progressBar.setGeometry(QtCore.QRect(20, 60, 118, 23))
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName(_fromUtf8("progressBar"))
        self.label_2 = QtGui.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(20, 20, 111, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(200, 20, 111, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(Form)
        self.label_4.setGeometry(QtCore.QRect(190, 60, 131, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setText(_fromUtf8(""))
        self.label_4.setObjectName(_fromUtf8("label_4"))

	self.table_no = 0
	self.current_table_position = 0
	self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	self.goal = MoveBaseGoal()



	self.update_values()


        self.retranslateUi(Form)
        QtCore.QObject.connect(self.spinBox, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.set_table_number)
        QtCore.QObject.connect(self.pushButton_3, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Home)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Go)
        QtCore.QObject.connect(self.pushButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Cancel)
        QtCore.QMetaObject.connectSlotsByName(Form)


    def set_table_number(self):
	self.table_no = self.spinBox.value()
	self.current_table_position = table_position[self.table_no]
	print self.current_table_position

    def Go(self):
	print "Go"


	print "Waiting for server"
#	self.client.wait_for_server()

	
	self.goal.target_pose.pose.position.x=float(self.current_table_position[0])
	self.goal.target_pose.pose.position.y=float(self.current_table_position[1])
	self.goal.target_pose.pose.position.z=float(self.current_table_position[2])

	self.goal.target_pose.pose.orientation.x = float(self.current_table_position[3])
	self.goal.target_pose.pose.orientation.y= float(self.current_table_position[4])
	self.goal.target_pose.pose.orientation.z= float(self.current_table_position[5])

	self.goal.target_pose.header.frame_id= 'map'
	self.goal.target_pose.header.stamp = rospy.Time.now()

	
#	print temp_table_pose[0]
#	print temp_table_pose[1]

	print "Go"

	self.client.send_goal(self.goal)

#	self.client.wait_for_result()
#	rospy.loginfo(self.client.get_result())



    def Cancel(self):
	print "Cancel"
	self.client.cancel_all_goals()




    def Home(self):
	print "Home"
	self.current_table_position = table_position[0]
	self.Go()


    def add(self,text):

	battery_value = rospy.get_param("battery_value")
	robot_status = rospy.get_param("robot_status")

        self.progressBar.setProperty("value", battery_value)
        self.label_4.setText(_fromUtf8(robot_status))
 


    def update_values(self):
  	self.thread =  WorkThread() 
  	QtCore.QObject.connect( self.thread,  QtCore.SIGNAL("update(QString)"), self.add )
  	self.thread.start()




    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Robot", None))
        self.label.setText(_translate("Form", "Table No(1-9)", None))
        self.pushButton.setText(_translate("Form", "Go", None))
        self.pushButton_2.setText(_translate("Form", "Cancel", None))
        self.pushButton_3.setText(_translate("Form", "Home", None))
        self.label_2.setText(_translate("Form", "Battery Level", None))
        self.label_3.setText(_translate("Form", "Robot Status", None))





class WorkThread(QtCore.QThread):
	def __init__(self):
		QtCore.QThread.__init__(self)
 
	def __del__(self):
		self.wait()
 
	def run(self):
		while True:
			time.sleep(0.3) # artificial time delay
			self.emit( QtCore.SIGNAL('update(QString)'), " " ) 
#			print "Hello"
	
   
  		return

if __name__ == "__main__":
	import sys



	rospy.init_node('robot_gui')
	rospy.set_param('battery_value',0)
	rospy.set_param('robot_status'," ")


	app = QtGui.QApplication(sys.argv)
	Form = QtGui.QWidget()
	ui = Ui_Form()
	ui.setupUi(Form)
	Form.show()
	sys.exit(app.exec_())

