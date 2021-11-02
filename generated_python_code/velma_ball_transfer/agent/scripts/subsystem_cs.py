#!/usr/bin/env python
'''
	Copyright (c) 2019, Robot Control and Pattern Recognition Group, Warsaw University of Technology
	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
				* Redistributions of source code must retain the above copyright
				  notice, this list of conditions and the following disclaimer.
				* Redistributions in binary form must reproduce the above copyright
				  notice, this list of conditions and the following disclaimer in the
				  documentation and/or other materials provided with the distribution.
				* Neither the name of the Warsaw University of Technology nor the
				  names of its contributors may be used to endorse or promote products
				  derived from this software without specific prior written permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	Author: Maksym Figat

'''


# Import other scripts #
from auxiliary_functions import *
from auxiliary_agent_agent import *
from auxiliary_subsystem_cs import *

# Temporary definitions #
IS_LOG = False # Flag determining if logs are shown in the terminal #
IS_PRINT = True # Flag indicating if debug information for developer are shown in the terminal #
class cs:
	##### Subsystem cs constructor #####
	def __init__(self):
		self.log("__init__ function")
		rospy.init_node("cs")
		self._subsystemName="cs"
		self._subsystemFrequency=10;
		self._currentSubsystemState="S_Idle";
		self._subsystemIterations=0
		self._behaviourIterations=0
		self.initialiseCommunicationModel()
		self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
		pass

	##### Start subsystem #####
	def startSubsystem(self):
		self.log("startSubsystem")
		try:
			while self.auxiliaryFunctions.isSubsystemOK():
				''' Execute behaviour associated with _currentSubsystemState -- choose appropriate state based on _currentSubsystemState '''
				if self._currentSubsystemState=="S_Idle":
					self.log("_currentSubsystemState==S_Idle")
					self.subsystemState_S_Idle()
					continue
				if self._currentSubsystemState=="S_InitializeRobotSystem":
					self.log("_currentSubsystemState==S_InitializeRobotSystem")
					self.subsystemState_S_InitializeRobotSystem()
					continue
				if self._currentSubsystemState=="S_MoveInFrontOfFirstTable":
					self.log("_currentSubsystemState==S_MoveInFrontOfFirstTable")
					self.subsystemState_S_MoveInFrontOfFirstTable()
					continue
				if self._currentSubsystemState=="S_TakeCupFromTable":
					self.log("_currentSubsystemState==S_TakeCupFromTable")
					self.subsystemState_S_TakeCupFromTable()
					continue
				if self._currentSubsystemState=="S_MoveInFrontOfThirdTable":
					self.log("_currentSubsystemState==S_MoveInFrontOfThirdTable")
					self.subsystemState_S_MoveInFrontOfThirdTable()
					continue
				if self._currentSubsystemState=="S_PutCupOnTable":
					self.log("_currentSubsystemState==S_PutCupOnTable")
					self.subsystemState_S_PutCupOnTable()
					continue
				if self._currentSubsystemState=="S_MoveInFrontOfSecondTable":
					self.log("_currentSubsystemState==S_MoveInFrontOfSecondTable")
					self.subsystemState_S_MoveInFrontOfSecondTable()
					continue
				if self._currentSubsystemState=="S_OpenDoor":
					self.log("_currentSubsystemState==S_OpenDoor")
					self.subsystemState_S_OpenDoor()
					continue
				if self._currentSubsystemState=="S_TakeCupFromCabinetAvoidingDoor":
					self.log("_currentSubsystemState==S_TakeCupFromCabinetAvoidingDoor")
					self.subsystemState_S_TakeCupFromCabinetAvoidingDoor()
					continue
				if self._currentSubsystemState=="S_MoveInFrontOfThirdTableAvoidingDoor":
					self.log("_currentSubsystemState==S_MoveInFrontOfThirdTableAvoidingDoor")
					self.subsystemState_S_MoveInFrontOfThirdTableAvoidingDoor()
					continue
				if self._currentSubsystemState=="S_PourBallOutOfCup":
					self.log("_currentSubsystemState==S_PourBallOutOfCup")
					self.subsystemState_S_PourBallOutOfCup()
					continue
				if self._currentSubsystemState=="S_Stop":
					self.log("_currentSubsystemState==S_Stop")
					self.subsystemState_S_Stop()
					continue
		except Exception as e:
			print e
			self.error("Error found in function startSubsystem -- file subsystem_cs.py!")
			pass

	##### Initialise communication model #####
	def initialiseCommunicationModel(self):
		self.log("initialiseCommunicationModel")
		self.initialiseSendChannel()
		self.initialiseSendChannelForDiagnostics()
		self.initialiseReceiveChannel()
		pass

	##### Initialise send channel #####
	def initialiseSendChannel(self):
		self.log("initialiseSendChannel")
		pass

	##### Initialise send channel for diagnostics #####
	def initialiseSendChannelForDiagnostics(self):
		self.log("initialiseSendChannelForDiagnostics")
		self._vectorOfSenderDiagnostics=[]
		self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_currentSubsystemState', String, queue_size=CHANNEL_SIZE))
		self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemFrequency', Float64, queue_size=CHANNEL_SIZE))
		self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemName', String, queue_size=CHANNEL_SIZE))
		self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemIterations', Int64, queue_size=CHANNEL_SIZE))
		self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_behaviourIterations', Int64, queue_size=CHANNEL_SIZE))
		pass

	##### Initialise receive channel based on input buffers #####
	def initialiseReceiveChannel(self):
		self.log("initialiseReceiveChannel")
		pass

	##### Wait for all messages #####
	def waitForAllMessages(self):
		self.log("waitForAllMessages")
		pass

	##### Publish on topics diagnostic data concerning the subsystem state #####
	def sendDataForDiagnostics(self):
		self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemState)
		self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
		self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
		self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
		self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
		pass

	##### Behaviour definitions #####

	##### Behaviour Idle #####
	##### Terminal condition #####
	def terminalCondition_Idle(self): # Int64_currentState #
		self.log("[Behaviour Idle] -- Checking Terminal Condition")
		return  True 
		pass

	##### Error condition #####
	def errorCondition_Idle(self): # Int64_currentState #
		self.log("[Behaviour Idle] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_Idle(self): # Int64_currentState #
		self.log("[Behaviour Idle] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - Idle")
		#  Partial transition function - name - tf1_2
		print "Do nothing - Idle behaviour"
		self._currentState=Int64(0)
		pass

	##### Send data to other subsystems #####
	def sendData_Idle(self):
		self.log("[Behaviour Idle] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour Idle] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_Idle(self):
		self.log("[Behaviour Idle] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour Idle] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour Idle #####
	def executeBehaviour_Idle(self):
		self.log("[Behaviour Idle] -- Executing Idle Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour Idle #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_Idle()
			# Sends data! #
			self.sendData_Idle()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_Idle()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_Idle() or self.errorCondition_Idle()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour InitializeRobotSystem #####
	##### Terminal condition #####
	def terminalCondition_InitializeRobotSystem(self): # Int64_currentState #
		self.log("[Behaviour InitializeRobotSystem] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_InitializeRobotSystem(self): # Int64_currentState #
		self.log("[Behaviour InitializeRobotSystem] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_InitializeRobotSystem(self): # Int64_currentState #
		self.log("[Behaviour InitializeRobotSystem] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - InitializeRobotSystem")
		#  Partial transition function - name - tf1_1
		self._currentState.data=STATE_INITIALIZE_ROBOT_SYSTEM
		print "initializeRobotSystem ", str(self._currentState.data)
		initialize_velma()
		initialize_planner()
		initialize_octomap()
		pass

	##### Send data to other subsystems #####
	def sendData_InitializeRobotSystem(self):
		self.log("[Behaviour InitializeRobotSystem] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour InitializeRobotSystem] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_InitializeRobotSystem(self):
		self.log("[Behaviour InitializeRobotSystem] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour InitializeRobotSystem] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour InitializeRobotSystem #####
	def executeBehaviour_InitializeRobotSystem(self):
		self.log("[Behaviour InitializeRobotSystem] -- Executing InitializeRobotSystem Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour InitializeRobotSystem #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_InitializeRobotSystem()
			# Sends data! #
			self.sendData_InitializeRobotSystem()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_InitializeRobotSystem()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_InitializeRobotSystem() or self.errorCondition_InitializeRobotSystem()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour MoveInFrontOfFirstTable #####
	##### Terminal condition #####
	def terminalCondition_MoveInFrontOfFirstTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Checking Terminal Condition")
		return ( False ) or (  True )
		pass

	##### Error condition #####
	def errorCondition_MoveInFrontOfFirstTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Checking Error Condition")
		return false
		pass

	##### Transition function #####
	def transitionFunction_MoveInFrontOfFirstTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - MoveInFrontOfFirstTable")
		self._currentState.data=STATE_MOVE_IN_FRONT_OF_FIRST_TABLE
		print "moveInFrontOfFirstTable ", str(self._currentState.data)
		rotate_torso(1.56) # rotate torso by 1.5 radian (left)
		move_robot_to_ready_position(False, False, LEFT_HAND)
		pass

	##### Send data to other subsystems #####
	def sendData_MoveInFrontOfFirstTable(self):
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_MoveInFrontOfFirstTable(self):
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour MoveInFrontOfFirstTable #####
	def executeBehaviour_MoveInFrontOfFirstTable(self):
		self.log("[Behaviour MoveInFrontOfFirstTable] -- Executing MoveInFrontOfFirstTable Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour MoveInFrontOfFirstTable #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_MoveInFrontOfFirstTable()
			# Sends data! #
			self.sendData_MoveInFrontOfFirstTable()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_MoveInFrontOfFirstTable()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_MoveInFrontOfFirstTable() or self.errorCondition_MoveInFrontOfFirstTable()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour TakeCupFromTable #####
	##### Terminal condition #####
	def terminalCondition_TakeCupFromTable(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromTable] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_TakeCupFromTable(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromTable] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_TakeCupFromTable(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromTable] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - TakeCupFromTable")
		#  Partial transition function - name - tf1_1
		self._currentState.data=STATE_TAKE_CUP_FROM_TABLE
		print "takeCupFromTable"
		switch_to_cartesian_impedance_mode(LEFT_HAND)
		move_in_front_of_cup(True, LEFT_HAND)
		print "take_cup_from_table"
		# open fingers
		print "Open fingers"
		how_wide_to_open=OPEN_HAND_FINGERS
		q_gripper=[how_wide_to_open, how_wide_to_open, how_wide_to_open, 0]
		set_grippers(LEFT_HAND,q_gripper )
		# move along y axis in order to set position of end-effector aligned to the cup (in Y axis)
		print "Move along X-axis"
		move_tool_by_delta(LEFT_HAND, 0.01, 0, 0, 0)
		# move a little bit up along Z axis
		print "Move along Z-axis"
		move_tool_by_delta(LEFT_HAND, 0, 0, 0.08, 0)
		# move along Y-axis
		print "Move along Y-axis"
		grip_frame=check_object_frame("B", "Gl")
		grip_pose=grip_frame.p
		print "Left grip frame pose: "+str(grip_pose)
		cup_frame=check_object_frame("B", "cup_on_table")
		cup_pose=cup_frame.p
		print "Cup pose: "+str(cup_pose)
		epsilon=0.005
		distance=cup_pose[1]-grip_pose[1]
		delta_y=distance/3
		print "Delta_y="+str(delta_y)
		move_tool_by_delta(LEFT_HAND, 0, 2*delta_y, 0, 0)
		delta_y=delta_y-epsilon
		print "Delta_y="+str(delta_y)
		move_tool_by_delta(LEFT_HAND, 0, delta_y, 0, 0)
		print "Close fingers"
		how_wide_to_open=CLOSE_HAND_FINGERS
		q_gripper=[how_wide_to_open, how_wide_to_open, how_wide_to_open, 0]
		set_grippers(LEFT_HAND,q_gripper )
		print "Move hand back along Y-axis"
		move_tool_by_delta(LEFT_HAND, -0.3, -distance-0.1, 0.03, 0)
		pass

	##### Send data to other subsystems #####
	def sendData_TakeCupFromTable(self):
		self.log("[Behaviour TakeCupFromTable] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour TakeCupFromTable] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_TakeCupFromTable(self):
		self.log("[Behaviour TakeCupFromTable] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour TakeCupFromTable] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour TakeCupFromTable #####
	def executeBehaviour_TakeCupFromTable(self):
		self.log("[Behaviour TakeCupFromTable] -- Executing TakeCupFromTable Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour TakeCupFromTable #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_TakeCupFromTable()
			# Sends data! #
			self.sendData_TakeCupFromTable()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_TakeCupFromTable()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_TakeCupFromTable() or self.errorCondition_TakeCupFromTable()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour MoveInFrontOfThirdTable #####
	##### Terminal condition #####
	def terminalCondition_MoveInFrontOfThirdTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_MoveInFrontOfThirdTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_MoveInFrontOfThirdTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - MoveInFrontOfThirdTable")
		#  Partial transition function - name - tf1_1
		print "moveInFrontOfThirdTable"
		self._currentState.data=STATE_MOVE_IN_FRONT_OF_THIRD_TABLE
		move_tool_by_delta(LEFT_HAND,0.2,0,0,-1.08)
		rotate_torso(0) # rotate torso to position 0
		rotate_torso(-1.56) # rotate torso to position -1.56
		pass

	##### Send data to other subsystems #####
	def sendData_MoveInFrontOfThirdTable(self):
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour MoveInFrontOfThirdTable] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_MoveInFrontOfThirdTable(self):
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour MoveInFrontOfThirdTable] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour MoveInFrontOfThirdTable #####
	def executeBehaviour_MoveInFrontOfThirdTable(self):
		self.log("[Behaviour MoveInFrontOfThirdTable] -- Executing MoveInFrontOfThirdTable Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour MoveInFrontOfThirdTable #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_MoveInFrontOfThirdTable()
			# Sends data! #
			self.sendData_MoveInFrontOfThirdTable()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_MoveInFrontOfThirdTable()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_MoveInFrontOfThirdTable() or self.errorCondition_MoveInFrontOfThirdTable()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour PutCupOnTable #####
	##### Terminal condition #####
	def terminalCondition_PutCupOnTable(self): # Int64_currentState #
		self.log("[Behaviour PutCupOnTable] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_PutCupOnTable(self): # Int64_currentState #
		self.log("[Behaviour PutCupOnTable] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_PutCupOnTable(self): # Int64_currentState #
		self.log("[Behaviour PutCupOnTable] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - PutCupOnTable")
		#  Partial transition function - name - tf1_1
		print "putCupOnTable"
		self._currentState.data=STATE_PUT_CUP_ON_TABLE
		switch_to_cartesian_impedance_mode(BOTH_HANDS)
		move_tool_by_delta(LEFT_HAND,-0.2,-0.3,0.1,0.8)
		move_tool_by_delta(LEFT_HAND,-0.35,-0.2,0.1,-0.2)
		move_tool_by_delta(LEFT_HAND,0,0,-0.15,0)
		q_gripper=[0, 0, 0, 0]
		set_grippers(LEFT_HAND,q_gripper )
		move_tool_by_delta(LEFT_HAND,0,0.2,0.2,0)
		pass

	##### Send data to other subsystems #####
	def sendData_PutCupOnTable(self):
		self.log("[Behaviour PutCupOnTable] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour PutCupOnTable] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_PutCupOnTable(self):
		self.log("[Behaviour PutCupOnTable] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour PutCupOnTable] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour PutCupOnTable #####
	def executeBehaviour_PutCupOnTable(self):
		self.log("[Behaviour PutCupOnTable] -- Executing PutCupOnTable Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour PutCupOnTable #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_PutCupOnTable()
			# Sends data! #
			self.sendData_PutCupOnTable()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_PutCupOnTable()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_PutCupOnTable() or self.errorCondition_PutCupOnTable()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour MoveInFrontOfSecondTable #####
	##### Terminal condition #####
	def terminalCondition_MoveInFrontOfSecondTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Checking Terminal Condition")
		return ( False ) or (  True )
		pass

	##### Error condition #####
	def errorCondition_MoveInFrontOfSecondTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Checking Error Condition")
		return false
		pass

	##### Transition function #####
	def transitionFunction_MoveInFrontOfSecondTable(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - MoveInFrontOfSecondTable")
		print "moveInFrontOfSecondTable"
		self._currentState.data=STATE_MOVE_IN_FRONT_OF_SECOND_TABLE
		set_grippers(LEFT_HAND, [1.57,1.57,1.57,3.14]) # for left arm
		move_tool_by_delta(LEFT_HAND,0.3,0,-0.3,0)
		move_tool_by_delta(LEFT_HAND,0.1,0.2,-0.2,0)
		move_robot_to_initial_position(False, BOTH_HANDS)
		move_robot_to_ready_position(True, True, RIGHT_HAND)
		print "move_to_cabinet_right_door"
		# change position of gripper
		q_gripper=[1.8, 1.8, 1.8, 3.14]
		set_grippers(RIGHT_HAND, q_gripper)
		# get position of cabinet right door with respect to base frame
		frame=check_object_frame("B", "cabinet_right_door")
		position=frame.p
		# position of grip frame
		delta_x=0.235426
		delta_z=0.07662
		tool_position=PyKDL.Vector(position[0],position[1],position[2])
		tool_position[0]=tool_position[0]-delta_x
		tool_position[2]=tool_position[2]+delta_z
		position=tool_position
		# move close to the door
		position=PyKDL.Vector(position[0]-0.1, position[1]+0.1, position[2])
		move_end_effector_to_cartesian_position_and_angle_and_wrench(RIGHT_HAND, 0,0,0, position[0], position[1], position[2], IMP_LIST_DEFAULT_STIFFNESS, IMP_TIME_LIST, MAX_WRENCH_DEFAULT, PATH_TOLERANCE_DEFAULT, DEFAULT_TIME)
		pass

	##### Send data to other subsystems #####
	def sendData_MoveInFrontOfSecondTable(self):
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_MoveInFrontOfSecondTable(self):
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour MoveInFrontOfSecondTable #####
	def executeBehaviour_MoveInFrontOfSecondTable(self):
		self.log("[Behaviour MoveInFrontOfSecondTable] -- Executing MoveInFrontOfSecondTable Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour MoveInFrontOfSecondTable #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_MoveInFrontOfSecondTable()
			# Sends data! #
			self.sendData_MoveInFrontOfSecondTable()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_MoveInFrontOfSecondTable()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_MoveInFrontOfSecondTable() or self.errorCondition_MoveInFrontOfSecondTable()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour OpenDoor #####
	##### Terminal condition #####
	def terminalCondition_OpenDoor(self): # Int64_currentState #
		self.log("[Behaviour OpenDoor] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_OpenDoor(self): # Int64_currentState #
		self.log("[Behaviour OpenDoor] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_OpenDoor(self): # Int64_currentState #
		self.log("[Behaviour OpenDoor] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - OpenDoor")
		#  Partial transition function - name - tf1_1
		print "openDoor"
		self._currentState.data=STATE_OPEN_DOOR
		switch_to_cartesian_impedance_mode(RIGHT_HAND)
		move_tool_by_delta(RIGHT_HAND, 0.05, 0, 0, 0)
		move_tool_by_delta(RIGHT_HAND, 0, 0.16, 0, 0)
		move_tool_by_delta(RIGHT_HAND, -0.05, -0.2, 0, 0.78)
		move_tool_by_delta(RIGHT_HAND, -0.05, -0.05, 0, 0.1)
		move_tool_by_delta(RIGHT_HAND, 0, -0.1, 0, 0.2)
		move_tool_by_delta(RIGHT_HAND, 0.03, -0.13, 0, 0.2)
		move_tool_by_delta(RIGHT_HAND, 0, -0.04, 0, 0)
		move_tool_by_delta(RIGHT_HAND, 0.05, 0, 0, -1.28)
		pass

	##### Send data to other subsystems #####
	def sendData_OpenDoor(self):
		self.log("[Behaviour OpenDoor] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour OpenDoor] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_OpenDoor(self):
		self.log("[Behaviour OpenDoor] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour OpenDoor] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour OpenDoor #####
	def executeBehaviour_OpenDoor(self):
		self.log("[Behaviour OpenDoor] -- Executing OpenDoor Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour OpenDoor #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_OpenDoor()
			# Sends data! #
			self.sendData_OpenDoor()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_OpenDoor()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_OpenDoor() or self.errorCondition_OpenDoor()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour TakeCupFromCabinetAvoidingDoor #####
	##### Terminal condition #####
	def terminalCondition_TakeCupFromCabinetAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_TakeCupFromCabinetAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_TakeCupFromCabinetAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - TakeCupFromCabinetAvoidingDoor")
		#  Partial transition function - name - tf1_1
		print "takeCupFromCabinetAvoidingDoor"
		self._currentState.data=STATE_TAKE_CUP_FROM_CABINET_AVOIDING_DOOR
		print "move_to_cabinet_right_door_after_openning_the_right_door"
		# move hand back in front of cabinet (avoid the right opened door)
		size_of_door=0.282
		move_tool_by_delta(RIGHT_HAND, -size_of_door, 0, 0, 0.98) # length of the right cabinet door and avoiding the opened right door
		move_tool_by_delta(RIGHT_HAND, 0.05, 0.3, 0, 0)
		move_tool_by_delta(RIGHT_HAND, 0.15, 0.15, 0, -0.98) # hand should be in front of cabinet
		move_in_front_of_cup(False, RIGHT_HAND)
		print "####### take_cup_from_cabinet #######"
		# open fingers
		print "Open fingers"
		how_wide_to_open=OPEN_HAND_FINGERS
		q_gripper=[how_wide_to_open, how_wide_to_open, how_wide_to_open, 0]
		set_grippers(RIGHT_HAND,q_gripper)
		# move along y axis in order to set position of end-effector aligned to the cup (in Y axis)
		print "Move along Y-axis"
		move_tool_by_delta(RIGHT_HAND, 0, 0.01, 0, 0)
		# move a little bit up along Z axis
		print "Move along Z-axis"
		move_tool_by_delta(RIGHT_HAND, 0, 0, 0.08, 0)
		# move along X-axis
		print "Move along X-axis"
		grip_frame=check_object_frame("B", "Gr")
		grip_pose=grip_frame.p
		print "Right grip frame pose: "+str(grip_pose)
		cup_frame=check_object_frame("B", "cup_in_cabinet")
		cup_pose=cup_frame.p
		print "Cup pose: "+str(cup_pose)
		epsilon=0.01
		distance=cup_pose[0]-grip_pose[0]
		delta_x=distance/3
		print "Delta_x="+str(delta_x)
		move_tool_by_delta(RIGHT_HAND, delta_x, 0, 0, 0)
		move_tool_by_delta(RIGHT_HAND, delta_x, 0, 0, 0)
		delta_x=delta_x-epsilon
		print "Delta_x="+str(delta_x)
		move_tool_by_delta(RIGHT_HAND, delta_x, 0, 0, 0)
		print "Close fingers"
		how_wide_to_open=CLOSE_HAND_FINGERS
		q_gripper=[how_wide_to_open, how_wide_to_open, how_wide_to_open, 0]
		set_grippers(RIGHT_HAND,q_gripper )
		print "Move along Z axis"
		move_tool_by_delta(RIGHT_HAND, 0, 0, 0.03, 0)
		print "Move hand back along X-axis"
		move_tool_by_delta(RIGHT_HAND, -distance, 0, 0, 0)
		move_tool_by_delta(RIGHT_HAND, -0.1, 0, 0, 0)
		pass

	##### Send data to other subsystems #####
	def sendData_TakeCupFromCabinetAvoidingDoor(self):
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour TakeCupFromCabinetAvoidingDoor] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_TakeCupFromCabinetAvoidingDoor(self):
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour TakeCupFromCabinetAvoidingDoor] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour TakeCupFromCabinetAvoidingDoor #####
	def executeBehaviour_TakeCupFromCabinetAvoidingDoor(self):
		self.log("[Behaviour TakeCupFromCabinetAvoidingDoor] -- Executing TakeCupFromCabinetAvoidingDoor Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour TakeCupFromCabinetAvoidingDoor #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_TakeCupFromCabinetAvoidingDoor()
			# Sends data! #
			self.sendData_TakeCupFromCabinetAvoidingDoor()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_TakeCupFromCabinetAvoidingDoor()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_TakeCupFromCabinetAvoidingDoor() or self.errorCondition_TakeCupFromCabinetAvoidingDoor()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour MoveInFrontOfThirdTableAvoidingDoor #####
	##### Terminal condition #####
	def terminalCondition_MoveInFrontOfThirdTableAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_MoveInFrontOfThirdTableAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_MoveInFrontOfThirdTableAvoidingDoor(self): # Int64_currentState #
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - MoveInFrontOfThirdTableAvoidingDoor")
		#  Partial transition function - name - tf1_1
		print "moveInFrontOfThirdTableAvoidingDoor"
		self._currentState.data=STATE_MOVE_IN_FRONT_OF_THIRD_TABLE_AVOIDING_DOOR
		switch_to_cartesian_impedance_mode(RIGHT_HAND)
		move_tool_by_delta(RIGHT_HAND,0,0,0,0.78)
		rotate_torso(-1.56) # rotate torso to position -1.56
		move_robot_to_initial_position(False, LEFT_HAND)
		pass

	##### Send data to other subsystems #####
	def sendData_MoveInFrontOfThirdTableAvoidingDoor(self):
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_MoveInFrontOfThirdTableAvoidingDoor(self):
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour MoveInFrontOfThirdTableAvoidingDoor #####
	def executeBehaviour_MoveInFrontOfThirdTableAvoidingDoor(self):
		self.log("[Behaviour MoveInFrontOfThirdTableAvoidingDoor] -- Executing MoveInFrontOfThirdTableAvoidingDoor Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour MoveInFrontOfThirdTableAvoidingDoor #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_MoveInFrontOfThirdTableAvoidingDoor()
			# Sends data! #
			self.sendData_MoveInFrontOfThirdTableAvoidingDoor()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_MoveInFrontOfThirdTableAvoidingDoor()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_MoveInFrontOfThirdTableAvoidingDoor() or self.errorCondition_MoveInFrontOfThirdTableAvoidingDoor()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour PourBallOutOfCup #####
	##### Terminal condition #####
	def terminalCondition_PourBallOutOfCup(self): # Int64_currentState #
		self.log("[Behaviour PourBallOutOfCup] -- Checking Terminal Condition")
		return   True 
		pass

	##### Error condition #####
	def errorCondition_PourBallOutOfCup(self): # Int64_currentState #
		self.log("[Behaviour PourBallOutOfCup] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_PourBallOutOfCup(self): # Int64_currentState #
		self.log("[Behaviour PourBallOutOfCup] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - PourBallOutOfCup")
		#  Partial transition function - name - tf1_1
		print "pourBallOutOfCup"
		self._currentState.data=STATE_POUR_BALL_OUT_OF_CUP
		# get position of right hand
		switch_to_cartesian_impedance_mode(RIGHT_HAND)
		frame=check_object_frame("B","Tr")
		pose=frame.p
		angle_x=frame.M.GetRPY()[0]
		angle_y=frame.M.GetRPY()[1]
		angle_z=frame.M.GetRPY()[2]
		move_end_effector_to_cartesian_position_and_angle_and_wrench(RIGHT_HAND, angle_x, angle_y, angle_z, pose[0], pose[1], pose[2], IMP_LIST_DEFAULT_STIFFNESS, IMP_TIME_LIST, MAX_WRENCH_DEFAULT, PATH_TOLERANCE_DEFAULT_2, DEFAULT_TIME)
		move_tool_by_delta(RIGHT_HAND,0,0,0,-0.78)
		move_tool_by_delta(RIGHT_HAND,0,0,0,-0.7)
		print "Move in front of cup"
		move_in_front_of_cup(True, RIGHT_HAND)
		move_tool_by_delta(RIGHT_HAND,-0.1,0,0.1,0)
		move_tool_by_delta(RIGHT_HAND,-0.05,0,0.03,0)
		print "Move along Y-axis"
		grip_frame=check_object_frame("B", "Gr")
		grip_pose=grip_frame.p
		cup_frame=check_object_frame("B", "cup_on_table")
		cup_pose=cup_frame.p
		distance_y=cup_pose[1]-grip_pose[1]
		delta_y=distance_y
		move_tool_by_delta(RIGHT_HAND, 0, delta_y, 0, 0)
		move_and_rotate_tool_by_delta(RIGHT_HAND,0,0,0,-1.57,0,0)
		grip_frame=check_object_frame("B", "Gr")
		grip_pose=grip_frame.p
		distance_x=cup_pose[0]-grip_pose[0]
		delta_x=distance_x
		move_and_rotate_tool_by_delta(RIGHT_HAND, distance_x-0.1, 0, 0.05, 0,0,0)
		move_and_rotate_tool_by_delta(RIGHT_HAND,0,0,0,-0.77,0,0)
		move_and_rotate_tool_by_delta(RIGHT_HAND,0,0,0,0.67,0,0)
		move_and_rotate_tool_by_delta(RIGHT_HAND, -0.1, 0, 0.05, 0,0,0)
		move_and_rotate_tool_by_delta(RIGHT_HAND,0,0,0,1.57,0,0)
		pass

	##### Send data to other subsystems #####
	def sendData_PourBallOutOfCup(self):
		self.log("[Behaviour PourBallOutOfCup] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour PourBallOutOfCup] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_PourBallOutOfCup(self):
		self.log("[Behaviour PourBallOutOfCup] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour PourBallOutOfCup] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour PourBallOutOfCup #####
	def executeBehaviour_PourBallOutOfCup(self):
		self.log("[Behaviour PourBallOutOfCup] -- Executing PourBallOutOfCup Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour PourBallOutOfCup #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_PourBallOutOfCup()
			# Sends data! #
			self.sendData_PourBallOutOfCup()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_PourBallOutOfCup()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_PourBallOutOfCup() or self.errorCondition_PourBallOutOfCup()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass

	##### Behaviour Stop #####
	##### Terminal condition #####
	def terminalCondition_Stop(self): # Int64_currentState #
		self.log("[Behaviour Stop] -- Checking Terminal Condition")
		return  True 
		pass

	##### Error condition #####
	def errorCondition_Stop(self): # Int64_currentState #
		self.log("[Behaviour Stop] -- Checking Error Condition")
		return  False 
		pass

	##### Transition function #####
	def transitionFunction_Stop(self): # Int64_currentState #
		self.log("[Behaviour Stop] -- Calculating Transition Function")
		# Transition function #
		self.log("TRANSITION FUNCTION - Stop")
		#  Partial transition function - name - tf1_1
		exit(1)
		pass

	##### Send data to other subsystems #####
	def sendData_Stop(self):
		self.log("[Behaviour Stop] -- Sending Data")
		# DIAGNOSTICS SEND #
		self.sendDataForDiagnostics()
		# END OF DIAGNOSTICS SEND #
		# TYPICAL SEND CALL #
		# END OF TYPICAL SEND CALL #

		# BEGIN OF BODY SEND CALL #
		print "[Behaviour Stop] -- Sending Data\n"
		# END OF BODY SEND CALL #
		pass

	##### Receive data from other subsystems #####
	def receiveData_Stop(self):
		self.log("[Behaviour Stop] -- Receiving Data")
		# TYPICAL RECEIVE CALL #
		self.waitForAllMessages() #
		# END OF TYPICAL RECEIVE CALL #
		# BEGIN OF RECEIVE BODY CALL #
		print "[Behaviour Stop] -- Receiving Data\n"
		# END OF RECEIVE BODY CALL #
		pass

	##### Execute behaviour Stop #####
	def executeBehaviour_Stop(self):
		self.log("[Behaviour Stop] -- Executing Stop Behaviour")
		stopBehaviourIteration=False
		# Execution of a single iteration of a behaviour Stop #
		_behaviourIterations=0
		# Starts execution! #
		while True:
			# Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
			self.auxiliaryFunctions.sleep()
			# Calculates transition function -- output and internal buffers can only be modified by this function! #
			self.transitionFunction_Stop()
			# Sends data! #
			self.sendData_Stop()
			# Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
			self._behaviourIterations=self._behaviourIterations+1
			# Receives data! #
			self.receiveData_Stop()
			# Check both conditions, i.e. terminal condition and error condition #
			stopBehaviourIteration = self.terminalCondition_Stop() or self.errorCondition_Stop()
			if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
				'''
				Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
				of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
				subsystem must have been switched to another state or SIGINT was sent
				'''
				break
			# Stops execution! #
		pass


	##### Definition of functions responsible for switching subsystem cs between states : S_Idle S_InitializeRobotSystem S_MoveInFrontOfFirstTable S_TakeCupFromTable S_MoveInFrontOfThirdTable S_PutCupOnTable S_MoveInFrontOfSecondTable S_OpenDoor S_TakeCupFromCabinetAvoidingDoor S_MoveInFrontOfThirdTableAvoidingDoor S_PourBallOutOfCup S_Stop  #####
	# State S_Idle: #
	def subsystemState_S_Idle(self):
		self.log("subsystemState_S_Idle")
		# Executing behaviour Idle #
		self.executeBehaviour_Idle()
		# Behaviour has been terminated #
		# Checking initial condition for state S_Idle: switching to state S_InitializeRobotSystem #
		if self.initialCondition_From_S_Idle_To_S_InitializeRobotSystem():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_InitializeRobotSystem"

		pass


	# State S_InitializeRobotSystem: #
	def subsystemState_S_InitializeRobotSystem(self):
		self.log("subsystemState_S_InitializeRobotSystem")
		# Executing behaviour InitializeRobotSystem #
		self.executeBehaviour_InitializeRobotSystem()
		# Behaviour has been terminated #
		# Checking initial condition for state S_InitializeRobotSystem: switching to state S_MoveInFrontOfFirstTable #
		if self.initialCondition_From_S_InitializeRobotSystem_To_S_MoveInFrontOfFirstTable():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_MoveInFrontOfFirstTable"

		pass


	# State S_MoveInFrontOfFirstTable: #
	def subsystemState_S_MoveInFrontOfFirstTable(self):
		self.log("subsystemState_S_MoveInFrontOfFirstTable")
		# Executing behaviour MoveInFrontOfFirstTable #
		self.executeBehaviour_MoveInFrontOfFirstTable()
		# Behaviour has been terminated #
		# Checking initial condition for state S_MoveInFrontOfFirstTable: switching to state S_TakeCupFromTable #
		if self.initialCondition_From_S_MoveInFrontOfFirstTable_To_S_TakeCupFromTable():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_TakeCupFromTable"

		pass


	# State S_TakeCupFromTable: #
	def subsystemState_S_TakeCupFromTable(self):
		self.log("subsystemState_S_TakeCupFromTable")
		# Executing behaviour TakeCupFromTable #
		self.executeBehaviour_TakeCupFromTable()
		# Behaviour has been terminated #
		# Checking initial condition for state S_TakeCupFromTable: switching to state S_MoveInFrontOfThirdTable #
		if self.initialCondition_From_S_TakeCupFromTable_To_S_MoveInFrontOfThirdTable():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_MoveInFrontOfThirdTable"

		pass


	# State S_MoveInFrontOfThirdTable: #
	def subsystemState_S_MoveInFrontOfThirdTable(self):
		self.log("subsystemState_S_MoveInFrontOfThirdTable")
		# Executing behaviour MoveInFrontOfThirdTable #
		self.executeBehaviour_MoveInFrontOfThirdTable()
		# Behaviour has been terminated #
		# Checking initial condition for state S_MoveInFrontOfThirdTable: switching to state S_PutCupOnTable #
		if self.initialCondition_From_S_MoveInFrontOfThirdTable_To_S_PutCupOnTable():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_PutCupOnTable"

		pass


	# State S_PutCupOnTable: #
	def subsystemState_S_PutCupOnTable(self):
		self.log("subsystemState_S_PutCupOnTable")
		# Executing behaviour PutCupOnTable #
		self.executeBehaviour_PutCupOnTable()
		# Behaviour has been terminated #
		# Checking initial condition for state S_PutCupOnTable: switching to state S_MoveInFrontOfSecondTable #
		if self.initialCondition_From_S_PutCupOnTable_To_S_MoveInFrontOfSecondTable():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_MoveInFrontOfSecondTable"

		pass


	# State S_MoveInFrontOfSecondTable: #
	def subsystemState_S_MoveInFrontOfSecondTable(self):
		self.log("subsystemState_S_MoveInFrontOfSecondTable")
		# Executing behaviour MoveInFrontOfSecondTable #
		self.executeBehaviour_MoveInFrontOfSecondTable()
		# Behaviour has been terminated #
		# Checking initial condition for state S_MoveInFrontOfSecondTable: switching to state S_OpenDoor #
		if self.initialCondition_From_S_MoveInFrontOfSecondTable_To_S_OpenDoor():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_OpenDoor"

		pass


	# State S_OpenDoor: #
	def subsystemState_S_OpenDoor(self):
		self.log("subsystemState_S_OpenDoor")
		# Executing behaviour OpenDoor #
		self.executeBehaviour_OpenDoor()
		# Behaviour has been terminated #
		# Checking initial condition for state S_OpenDoor: switching to state S_TakeCupFromCabinetAvoidingDoor #
		if self.initialCondition_From_S_OpenDoor_To_S_TakeCupFromCabinetAvoidingDoor():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_TakeCupFromCabinetAvoidingDoor"

		pass


	# State S_TakeCupFromCabinetAvoidingDoor: #
	def subsystemState_S_TakeCupFromCabinetAvoidingDoor(self):
		self.log("subsystemState_S_TakeCupFromCabinetAvoidingDoor")
		# Executing behaviour TakeCupFromCabinetAvoidingDoor #
		self.executeBehaviour_TakeCupFromCabinetAvoidingDoor()
		# Behaviour has been terminated #
		# Checking initial condition for state S_TakeCupFromCabinetAvoidingDoor: switching to state S_MoveInFrontOfThirdTableAvoidingDoor #
		if self.initialCondition_From_S_TakeCupFromCabinetAvoidingDoor_To_S_MoveInFrontOfThirdTableAvoidingDoor():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_MoveInFrontOfThirdTableAvoidingDoor"

		pass


	# State S_MoveInFrontOfThirdTableAvoidingDoor: #
	def subsystemState_S_MoveInFrontOfThirdTableAvoidingDoor(self):
		self.log("subsystemState_S_MoveInFrontOfThirdTableAvoidingDoor")
		# Executing behaviour MoveInFrontOfThirdTableAvoidingDoor #
		self.executeBehaviour_MoveInFrontOfThirdTableAvoidingDoor()
		# Behaviour has been terminated #
		# Checking initial condition for state S_MoveInFrontOfThirdTableAvoidingDoor: switching to state S_PourBallOutOfCup #
		if self.initialCondition_From_S_MoveInFrontOfThirdTableAvoidingDoor_To_S_PourBallOutOfCup():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_PourBallOutOfCup"

		pass


	# State S_PourBallOutOfCup: #
	def subsystemState_S_PourBallOutOfCup(self):
		self.log("subsystemState_S_PourBallOutOfCup")
		# Executing behaviour PourBallOutOfCup #
		self.executeBehaviour_PourBallOutOfCup()
		# Behaviour has been terminated #
		# Checking initial condition for state S_PourBallOutOfCup: switching to state S_Stop #
		if self.initialCondition_From_S_PourBallOutOfCup_To_S_Stop():
			# incrementing the number determining how many times subsystem has switched between states #
			self._subsystemIterations=self._subsystemIterations+1
			self._currentSubsystemState="S_Stop"

		pass


	# State S_Stop: #
	def subsystemState_S_Stop(self):
		self.log("subsystemState_S_Stop")
		# Executing behaviour Stop #
		self.executeBehaviour_Stop()
		# Behaviour has been terminated #
		pass


	##### Initial condition for state S_Idle: switching to state S_InitializeRobotSystem #####
	def initialCondition_From_S_Idle_To_S_InitializeRobotSystem(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_InitializeRobotSystem: switching to state S_MoveInFrontOfFirstTable #####
	def initialCondition_From_S_InitializeRobotSystem_To_S_MoveInFrontOfFirstTable(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_MoveInFrontOfFirstTable: switching to state S_TakeCupFromTable #####
	def initialCondition_From_S_MoveInFrontOfFirstTable_To_S_TakeCupFromTable(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_TakeCupFromTable: switching to state S_MoveInFrontOfThirdTable #####
	def initialCondition_From_S_TakeCupFromTable_To_S_MoveInFrontOfThirdTable(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_MoveInFrontOfThirdTable: switching to state S_PutCupOnTable #####
	def initialCondition_From_S_MoveInFrontOfThirdTable_To_S_PutCupOnTable(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_PutCupOnTable: switching to state S_MoveInFrontOfSecondTable #####
	def initialCondition_From_S_PutCupOnTable_To_S_MoveInFrontOfSecondTable(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_MoveInFrontOfSecondTable: switching to state S_OpenDoor #####
	def initialCondition_From_S_MoveInFrontOfSecondTable_To_S_OpenDoor(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_OpenDoor: switching to state S_TakeCupFromCabinetAvoidingDoor #####
	def initialCondition_From_S_OpenDoor_To_S_TakeCupFromCabinetAvoidingDoor(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_TakeCupFromCabinetAvoidingDoor: switching to state S_MoveInFrontOfThirdTableAvoidingDoor #####
	def initialCondition_From_S_TakeCupFromCabinetAvoidingDoor_To_S_MoveInFrontOfThirdTableAvoidingDoor(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_MoveInFrontOfThirdTableAvoidingDoor: switching to state S_PourBallOutOfCup #####
	def initialCondition_From_S_MoveInFrontOfThirdTableAvoidingDoor_To_S_PourBallOutOfCup(self):
		# Initial condition specified by user #
		return  True ;


	##### Initial condition for state S_PourBallOutOfCup: switching to state S_Stop #####
	def initialCondition_From_S_PourBallOutOfCup_To_S_Stop(self):
		# Initial condition specified by user #
		return  True ;



	##### Function indicating basic log/debug message #####
	def log(self, str):
		if(IS_LOG):
			rospy.loginfo("[SUBSYSTEM cs] -- LOG -- "+str)
		if(IS_PRINT):
			print "[SUBSYSTEM cs] -- LOG -- " + str
		pass

	##### Function indicating basic error message #####
	def error(self, str):
		sys.stderr.write("[SUBSYSTEM cs] -- ERROR -- " + str)
		if(IS_LOG):
			rospy.loginfo("[SUBSYSTEM cs] -- ERROR -- "+str)
			sys.exit()
		pass

##### MAIN FUNCTION FOR SUBSYSTEM cs #####
if __name__ == '__main__':
	try:
		subsystem_cs = cs()
		subsystem_cs.startSubsystem()
	except rospy.ROSInterruptException:
		pass

