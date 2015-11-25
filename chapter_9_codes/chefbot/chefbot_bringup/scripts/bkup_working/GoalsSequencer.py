#!/usr/bin/env python
'''
Created January, 2012

@author: Dr. Rainer Hessmer

  DriveToGoals.py - This ROS node reads a sequence of goals from a text file
  and then commands a robot running the ROS navigation stack to navigate to the
  goals in sequence.

  Copyright (c) 2012 Dr. Rainer Hessmer.  All right reserved.

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
import sys

# Brings in the SimpleActionClient
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion


class GoalsParser(object):
	''' 
	Helper class for extracting goals from a text file that contains goals either in simplified form or as recorded output
	from ros topic /move_base/goal. In case of the former format the class delegates the actual parsing to the class
	SimpleGoalsFileParser, for the latter it uses the class RecordedGoalsParser.
	'''

	@staticmethod
	def Parse(filePath):
		usesRecordedGoalsFormat = False
		with open(filePath, 'r') as file:
			line = file.readline()
			trimmedLine = line.strip()
			if trimmedLine == 'header:':
				usesRecordedGoalsFormat = True

		if usesRecordedGoalsFormat:
			goalsFileParser = RecordedGoalsParser()
			(frameId, goals) = goalsFileParser.Parse(goalsFilePath)
			return (frameId, goals)
		else:
			goalsFileParser = SimpleGoalsFileParser()
			(frameId, goals) = goalsFileParser.Parse(goalsFilePath)
			return (frameId, goals)


class RecordedGoalsParser(object):
	'''
	Helper class for extracting goals from a text file that contains the output of the
	ros topic /move_base/goal:
	
	rostopic echo /move_base/goal > ./goals.txt
	
	Content looks like this:
	
	header: 
	  seq: 5
	  stamp: 
		secs: 1327888889
		nsecs: 905062316
	  frame_id: ''
	goal_id: 
	  stamp: 
		secs: 0
		nsecs: 0
	  id: ''
	goal: 
	  target_pose: 
		header: 
		  seq: 5
		  stamp: 
			secs: 1327888889
			nsecs: 904623411
		  frame_id: /map
		pose: 
		  position: 
			x: 0.500366389751
			y: 3.55791568756
			z: 0.0
		  orientation: 
			x: 0.0
			y: 0.0
			z: -0.724127701373
			w: 0.689665913398
	---
	header: 
	  seq: 6
	  stamp: 
		secs: 1327888899
		nsecs: 302587125
	  frame_id: ''
	goal_id: 
	  stamp: 
		secs: 0
		nsecs: 0
	  id: ''
	goal: 
	  target_pose: 
		header: 
		  seq: 6
		  stamp: 
			secs: 1327888899
			nsecs: 302292358
		  frame_id: /map
		pose: 
		  position: 
			x: 0.353558450937
			y: -0.0242511983961
			z: 0.0
		  orientation: 
			x: 0.0
			y: 0.0
			z: 0.700985642079
			w: 0.713175384881
	---


	'''

	def Parse(self, filePath):
		'''
		Parses the specified file and returns the extracted frame id and the array of goal poses:
		(frame_id, [(x,y,theta)])
		'''
		
		self._GoalFrameId = '/map'
		self._GoalsFilePath = filePath
		with open(filePath, 'r') as file:
			goals = []
	
			while True:
				goal = self._ReadNextGoalSection(file)
				if goal is None:
					break
		
				goals.append(goal)
				# the next goal section starts after the line containing '---' which acts as a separater
				file.readline()
			
		return (self._GoalFrameId, goals)

	def _ReadNextGoalSection(self, file):
		'''
		Reads a section of the file that needs to be structured like this:
		
		header: 
		  seq: 5
		  stamp: 
			secs: 1327888889
			nsecs: 905062316
		  frame_id: ''
		goal_id: 
		  stamp: 
			secs: 0
			nsecs: 0
		  id: ''
		goal: 
		  target_pose: 
			header: 
			  seq: 5
			  stamp: 
				secs: 1327888889
				nsecs: 904623411
			  frame_id: /map
			pose: 
			  position: 
				x: 0.500366389751
				y: 3.55791568756
				z: 0.0
			  orientation: 
				x: 0.0
				y: 0.0
				z: -0.724127701373
				w: 0.689665913398

		'''
		
		# skip the first 18 lines
		for i in range(18):
			line = file.readline()
			if len(line) == 0:
				return None
		
		# next line contains the frame_id:
		line = file.readline()
		#print(line)
		lineParts = line.split(':')
		self._GoalFrameId = lineParts[1].strip()
		#print(self._GoalFrameId)
		
		file.readline()
		file.readline()

		# next line contains the x position
		line = file.readline()
		#print(line)
		x = self._ExtractValue('x', line)

		line = file.readline()
		y = self._ExtractValue('y', line)
		#print(line)
		
		file.readline() # z position
		file.readline() # orientation
		file.readline() # x orientation
		file.readline() # y orientation
		
		line = file.readline() # z orientation
		#print(line)
		theta = self._ExtractValue('z', line)

		file.readline() # w orientation

		goal = (x, y, theta)
		return goal
	
	def _ExtractValue(self, variableName, linePart):
		'''
		Takes as input text like this:
		     x: 0.500366389751
		
		Checks that the specified variableName matches the name of the variable in the string.
		then extracts the float value after the ':'
		'''

		nameValueParts = linePart.split(':')
		if nameValueParts[0].strip() != variableName:
			raise NameError('Expected variable name ' + variableName + ' but found ' + nameValueParts[0].strip())

		return float(nameValueParts[1].strip())


class SimpleGoalsFileParser(object):
	'''
	Helper class for extracting goals from a text file. Here is a sample file content:

	frame_id: /map
	// End of first hall way leg
	x: 0.705669820309, y: 3.92879199982, theta: -0.712161728691
	x: 2.3, y: 0.4, theta: 1.1

	'''

	def Parse(self, filePath):
		'''
		Parses the specified file and returns the found frame id and goal poses as a tuple of the form
		(frameId, [(x,y,theta)])
		'''
		self._GoalsFilePath = filePath
		with open(filePath, 'r') as file:
			frameId = None
			goals = []
			for line in file:
				trimmedLine = line.strip()
				if self._IsCommentOrEmpty(trimmedLine):
					continue
			
				if frameId is None:
					frameId = self._ParseFrameId(trimmedLine)
				else:
					goal = self._ParseGoalLine(trimmedLine)
					goals.append(goal)

		return (frameId, goals)

	def _IsCommentOrEmpty(self, trimmedLine):
		if trimmedLine.startswith('//'):
			# we are dealing with a comment line
			return True
		if len(trimmedLine) == 0:
			# we are dealing with an empty line
			return True
		return False

	def _ParseFrameId(self, trimmedLine):
		'''
		Takes as input text like this:
		frame_id: /map
		'''

		nameValueParts = trimmedLine.split(':')
		if nameValueParts[0].strip() != 'frame_id':
			raise NameError('Expected variable name frame_id but found ' + nameValueParts[0].strip())
		return nameValueParts[1].strip()

	def _ParseGoalLine(self, trimmedLine):
		#print(trimmedLine)
		lineParts = trimmedLine.split(',')
		x = self._ExtractValue('x', lineParts[0])
		y = self._ExtractValue('y', lineParts[1])
		theta = self._ExtractValue('theta', lineParts[2])
		
		goal = (x, y, theta)
		return goal
	
	def _ExtractValue(self, variableName, linePart):
		'''
		Takes as input text like this:
		x: 0.73444
		
		Checks that the specified variableName matches the name of the variable in the string.
		then extracts the float value of the '=' sign
		
		'''

		nameValueParts = linePart.split(':')
		if nameValueParts[0].strip() != variableName:
			raise NameError('Expected variable name ' + variableName + ' but found ' + nameValueParts[0].strip())

		return float(nameValueParts[1].strip())


class GoalsSequencer(object):
	'''
	'''
	
	def __init__(self, goalFrameId = '/map'):
		self._GoalFrameId = goalFrameId
		
		# Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
		rospy.init_node('goalsSequencer')
		
		self._Client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		# Waits until the action server has started up and started listening for goals.
		self._Client.wait_for_server()

	def NavigateToGoals(self, goals):
		for goal in goals:
			print("Navigating to goal " + str(goal))
			self._NavigateToGoal(goal)
		
	def _NavigateToGoal(self, goal):
		moveBaseGoal = self._CreateMoveBaseGoal(goal)
		
		self._Client.send_goal(moveBaseGoal)
		self._Client.wait_for_result()
		if self._Client.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("Goal reached")
		else:
			rospy.logerr("Could not execute goal for some reason")

	def _CreateMoveBaseGoal(self, goal):
		'''
		Creates an instance of MoveBaseGoal based on a simple goal in the form of a (x,y,theta) tuple
		'''
		
		x,y,theta = goal
		
		moveBaseGoal = MoveBaseGoal()
		moveBaseGoal.target_pose.header.frame_id = self._GoalFrameId
		moveBaseGoal.target_pose.header.stamp = rospy.Time.now()

		moveBaseGoal.target_pose.pose.position.x = x
		moveBaseGoal.target_pose.pose.position.y = y
		
		quaternionArray = tf.transformations.quaternion_about_axis(theta, (0,0,1))
		# quaternion_about_axis offers a convenient way for calculating the members of a quaternion.
		# In order to use it we need to convert it to a Quaternion message structure
		moveBaseGoal.target_pose.pose.orientation = self.array_to_quaternion(quaternionArray)

		print(moveBaseGoal)
		return moveBaseGoal

	def array_to_quaternion(self, nparr):
		'''
		Takes a numpy array holding the members of a quaternion and returns it as a 
		geometry_msgs.msg.Quaternion message instance.
		'''
		quat = Quaternion()
		quat.x = nparr[0]
		quat.y = nparr[1]
		quat.z = nparr[2]
		quat.w = nparr[3]
		return quat

if __name__ == '__main__':
	
	goalsFilePath = "./nodes/goals.txt"
	if (len(sys.argv) > 1):
		# we accept the path to the goals text file as a command line argument
		goalsFilePath = sys.argv[1]


	(frameId, goals) = GoalsParser.Parse(goalsFilePath)
	print((frameId, goals))

	goalsSequencer = GoalsSequencer(goalFrameId = frameId)
	goalsSequencer.NavigateToGoals(goals)
	
