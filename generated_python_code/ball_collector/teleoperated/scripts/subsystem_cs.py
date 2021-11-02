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
from auxiliary_agent_teleoperated import *
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
    self._currentSubsystemBehaviour="Behaviour_initBehaviour";
    self._subsystemIterations=0
    self._behaviourIterations=0
    self.initialiseCommunicationModel()
    self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
    # initialize all input flags
    self._in_flag_taskCommand=False
    self._in_flag_robotStatus=False
    self._in_flag_ballInfoTele=False
    self._in_flag_obstacleDetectedTele=False
    self._in_flag_ballCollected=False
    # initialize all output flags
    self._out_flag_taskStatus=False
    self._out_flag_desiredRobotCommandTele=False
    pass

  ##### Start subsystem #####
  def startSubsystem(self):
    self.log("startSubsystem")
    try:
      while self.auxiliaryFunctions.isSubsystemOK():
        ''' Execute behaviour associated with _currentSubsystemBehaviour -- choose appropriate state based on _currentSubsystemBehaviour '''
        if self._currentSubsystemBehaviour=="Behaviour_initBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_initBehaviour")
          self.subsystemBehaviour_initBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_idleBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_idleBehaviour")
          self.subsystemBehaviour_idleBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_terminateBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_terminateBehaviour")
          self.subsystemBehaviour_terminateBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_terminatedBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_terminatedBehaviour")
          self.subsystemBehaviour_terminatedBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_stopBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_stopBehaviour")
          self.subsystemBehaviour_stopBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveFasterBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveFasterBehaviour")
          self.subsystemBehaviour_moveFasterBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveSlowerBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveSlowerBehaviour")
          self.subsystemBehaviour_moveSlowerBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_vacuumOnBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_vacuumOnBehaviour")
          self.subsystemBehaviour_vacuumOnBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_vacuumOffBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_vacuumOffBehaviour")
          self.subsystemBehaviour_vacuumOffBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveBackwardBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveBackwardBehaviour")
          self.subsystemBehaviour_moveBackwardBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveFrontBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveFrontBehaviour")
          self.subsystemBehaviour_moveFrontBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveRightBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveRightBehaviour")
          self.subsystemBehaviour_moveRightBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_moveLeftBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_moveLeftBehaviour")
          self.subsystemBehaviour_moveLeftBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_rotateLeftBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_rotateLeftBehaviour")
          self.subsystemBehaviour_rotateLeftBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_rotateRightBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_rotateRightBehaviour")
          self.subsystemBehaviour_rotateRightBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_whatCanYouSeeBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_whatCanYouSeeBehaviour")
          self.subsystemBehaviour_whatCanYouSeeBehaviour()
          continue
    except Exception as e:
      print e
      self.error("Error found in function startSubsystem -- file subsystem_cs.py!")
      pass

  ##### Update data for input buffer: taskCommand #####
  def update_taskCommand(self, data):
    self.log("update_taskCommand")
    self.taskCommand=data
    self._in_flag_taskCommand=True
    pass

  ##### Update data for input buffer: ballInfoTele #####
  def update_ballInfoTele(self, data):
    self.log("update_ballInfoTele")
    self.ballInfoTele=data
    self._in_flag_ballInfoTele=True
    pass

  ##### Update data for input buffer: obstacleDetectedTele #####
  def update_obstacleDetectedTele(self, data):
    self.log("update_obstacleDetectedTele")
    self.obstacleDetectedTele=data
    self._in_flag_obstacleDetectedTele=True
    pass

  ##### Update data for input buffer: ballCollected #####
  def update_ballCollected(self, data):
    self.log("update_ballCollected")
    self.ballCollected=data
    self._in_flag_ballCollected=True
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
    # Buffer name=taskStatus - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_taskStatus=rospy.Publisher("taskStatusChannelTele", String, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredRobotCommandTele - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredRobotCommandTele=rospy.Publisher("desiredRobotCommandChannelTele", String, queue_size=CHANNEL_SIZE)
    pass

  ##### Initialise send channel for diagnostics #####
  def initialiseSendChannelForDiagnostics(self):
    self.log("initialiseSendChannelForDiagnostics")
    self._vectorOfSenderDiagnostics=[]
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_currentSubsystemBehaviour', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemFrequency', Float64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemName', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_subsystemIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_behaviourIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/isNewCommandReceived', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_taskCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_robotStatus', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_ballInfoTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_obstacleDetectedTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_ballCollected', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_taskStatus', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredRobotCommandTele', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=taskCommand sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_taskCommand=rospy.Subscriber("taskCommandChannel", String, self.update_taskCommand)
    # Buffer name=ballInfoTele sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_ballInfoTele=rospy.Subscriber("ballInfoChannelTele", CameraMessage, self.update_ballInfoTele)
    # Buffer name=obstacleDetectedTele sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_obstacleDetectedTele=rospy.Subscriber("obstacleDetectedChannelTele", ObstacleDetected, self.update_obstacleDetectedTele)
    # Buffer name=ballCollected sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_ballCollected=rospy.Subscriber("ballCollectedChannelTele", Bool, self.update_ballCollected)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", CameraMessage,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", ObstacleDetected,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Bool,  timeout=TOPIC_TIMEOUT)
    pass

  ##### Publish on topics diagnostic data concerning the subsystem state #####
  def sendDataForDiagnostics(self):
    self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemBehaviour)
    self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
    self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
    self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
    self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
    ###### internal state #####
    if(12 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self.isNewCommandReceived)
      self._vectorOfSenderDiagnostics[6].publish(self._in_flag_taskCommand)
      self._vectorOfSenderDiagnostics[7].publish(self._in_flag_robotStatus)
      self._vectorOfSenderDiagnostics[8].publish(self._in_flag_ballInfoTele)
      self._vectorOfSenderDiagnostics[9].publish(self._in_flag_obstacleDetectedTele)
      self._vectorOfSenderDiagnostics[10].publish(self._in_flag_ballCollected)
      self._vectorOfSenderDiagnostics[11].publish(self._out_flag_taskStatus)
      self._vectorOfSenderDiagnostics[12].publish(self._out_flag_desiredRobotCommandTele)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour initBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_initBehaviour(self): 
    self.log("[Behaviour initBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - initBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_initBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_initBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_initBehaviour_fun1(self): 
    self.log("[Behaviour initBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - initBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_initBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_initBehaviour_fun1_0(self): 
    self.log("[Behaviour initBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - initBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - teleoperated] -- initBehaviour")
    self.taskCommand=String("empty")
    self.taskStatus=String("empty")
    self.desiredRobotCommandTele=String("empty")
    self.robotStatus=String("empty")
    self.isNewCommandReceived=Int64(0)
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_initBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour initBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - initBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_initBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_initBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour initBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - initBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour initBehaviour #####
  def executeBehaviour_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Executing initBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour initBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_initBehaviour()
      # Sends data! #
      self.sendData_initBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_initBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_initBehaviour() or self.errorCondition_initBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour idleBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_idleBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  checkCommand(self.taskCommand.data) and self.isNewCommandReceived.data>0 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour idleBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_idleBehaviour(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - idleBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_idleBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_idleBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_taskCommand:
      self.transitionFunction_idleBehaviour_fun1_0()
    elif  not (self._in_flag_taskCommand):
      self.transitionFunction_idleBehaviour_fun1_1()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_taskCommand #####
  def transitionFunction_idleBehaviour_fun1_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - teleoperated] -- idleBehaviour - TRUE")
    self.isNewCommandReceived.data=1
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not (self._in_flag_taskCommand) #####
  def transitionFunction_idleBehaviour_fun1_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - teleoperated] -- idleBehaviour - FALSE")
    self.isNewCommandReceived.data=0
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_idleBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_idleBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_taskCommand=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour idleBehaviour #####
  def executeBehaviour_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Executing idleBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour idleBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_idleBehaviour()
      # Sends data! #
      self.sendData_idleBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_idleBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_idleBehaviour() or self.errorCondition_idleBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour terminateBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_terminateBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour terminateBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_terminateBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour terminateBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_terminateBehaviour(self): 
    self.log("[Behaviour terminateBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - terminateBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_terminateBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_terminateBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_terminateBehaviour_fun1(self): 
    self.log("[Behaviour terminateBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - terminateBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_terminateBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_terminateBehaviour_fun1_0(self): 
    self.log("[Behaviour terminateBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - terminateBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- terminateBehaviour")
    if(self.taskStatus.data=="task terminated"):
      self._out_flag_taskStatus=False
      self._out_flag_desiredRobotCommandTele=False
    else:
      self.taskStatus.data="task terminated"
      self.desiredRobotCommandTele.data="terminate"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_terminateBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour terminateBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - terminateBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_terminateBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_terminateBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour terminateBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - terminateBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_terminateBehaviour(self):
    self.log("[Behaviour terminateBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_terminateBehaviour(self):
    self.log("[Behaviour terminateBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour terminateBehaviour #####
  def executeBehaviour_terminateBehaviour(self):
    self.log("[Behaviour terminateBehaviour] -- Executing terminateBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour terminateBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_terminateBehaviour()
      # Sends data! #
      self.sendData_terminateBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_terminateBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_terminateBehaviour() or self.errorCondition_terminateBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour terminatedBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_terminatedBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour terminatedBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_terminatedBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour terminatedBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_terminatedBehaviour(self): 
    self.log("[Behaviour terminatedBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - terminatedBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_terminatedBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_terminatedBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_terminatedBehaviour_fun1(self): 
    self.log("[Behaviour terminatedBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - terminatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_terminatedBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_terminatedBehaviour_fun1_0(self): 
    self.log("[Behaviour terminatedBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - terminatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - teleoperated] -- terminatedBehaviour")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_terminatedBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour terminatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - terminatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_terminatedBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_terminatedBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour terminatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - terminatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_terminatedBehaviour(self):
    self.log("[Behaviour terminatedBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_terminatedBehaviour(self):
    self.log("[Behaviour terminatedBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour terminatedBehaviour #####
  def executeBehaviour_terminatedBehaviour(self):
    self.log("[Behaviour terminatedBehaviour] -- Executing terminatedBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour terminatedBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_terminatedBehaviour()
      # Sends data! #
      self.sendData_terminatedBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_terminatedBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_terminatedBehaviour() or self.errorCondition_terminatedBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour stopBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_stopBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour stopBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_stopBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour stopBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_stopBehaviour(self): 
    self.log("[Behaviour stopBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - stopBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_stopBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_stopBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_stopBehaviour_fun1(self): 
    self.log("[Behaviour stopBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - stopBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_stopBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_stopBehaviour_fun1_0(self): 
    self.log("[Behaviour stopBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - stopBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- stopBehaviour")
    self.desiredRobotCommandTele.data="stop"
    self.taskStatus.data="task status: stop robot"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_stopBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour stopBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - stopBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_stopBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_stopBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour stopBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - stopBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_stopBehaviour(self):
    self.log("[Behaviour stopBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_stopBehaviour(self):
    self.log("[Behaviour stopBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour stopBehaviour #####
  def executeBehaviour_stopBehaviour(self):
    self.log("[Behaviour stopBehaviour] -- Executing stopBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour stopBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_stopBehaviour()
      # Sends data! #
      self.sendData_stopBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_stopBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_stopBehaviour() or self.errorCondition_stopBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveFasterBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveFasterBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveFasterBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveFasterBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveFasterBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveFasterBehaviour(self): 
    self.log("[Behaviour moveFasterBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveFasterBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveFasterBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveFasterBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveFasterBehaviour_fun1(self): 
    self.log("[Behaviour moveFasterBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveFasterBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveFasterBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveFasterBehaviour_fun1_0(self): 
    self.log("[Behaviour moveFasterBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveFasterBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- moveFasterBehaviour")
    self.desiredRobotCommandTele.data="move faster"
    self.taskStatus.data="task status: move faster"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveFasterBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveFasterBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveFasterBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveFasterBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveFasterBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveFasterBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveFasterBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveFasterBehaviour(self):
    self.log("[Behaviour moveFasterBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveFasterBehaviour(self):
    self.log("[Behaviour moveFasterBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveFasterBehaviour #####
  def executeBehaviour_moveFasterBehaviour(self):
    self.log("[Behaviour moveFasterBehaviour] -- Executing moveFasterBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveFasterBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveFasterBehaviour()
      # Sends data! #
      self.sendData_moveFasterBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveFasterBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveFasterBehaviour() or self.errorCondition_moveFasterBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveSlowerBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveSlowerBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveSlowerBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveSlowerBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveSlowerBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveSlowerBehaviour(self): 
    self.log("[Behaviour moveSlowerBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveSlowerBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveSlowerBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveSlowerBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveSlowerBehaviour_fun1(self): 
    self.log("[Behaviour moveSlowerBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveSlowerBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveSlowerBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveSlowerBehaviour_fun1_0(self): 
    self.log("[Behaviour moveSlowerBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveSlowerBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- moveSlowerBehaviour")
    self.desiredRobotCommandTele.data="move slower"
    self.taskStatus.data="task status: move slower"  
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveSlowerBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveSlowerBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveSlowerBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveSlowerBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveSlowerBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveSlowerBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveSlowerBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveSlowerBehaviour(self):
    self.log("[Behaviour moveSlowerBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveSlowerBehaviour(self):
    self.log("[Behaviour moveSlowerBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveSlowerBehaviour #####
  def executeBehaviour_moveSlowerBehaviour(self):
    self.log("[Behaviour moveSlowerBehaviour] -- Executing moveSlowerBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveSlowerBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveSlowerBehaviour()
      # Sends data! #
      self.sendData_moveSlowerBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveSlowerBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveSlowerBehaviour() or self.errorCondition_moveSlowerBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour vacuumOnBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_vacuumOnBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour vacuumOnBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_vacuumOnBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour vacuumOnBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_vacuumOnBehaviour(self): 
    self.log("[Behaviour vacuumOnBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - vacuumOnBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_vacuumOnBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_vacuumOnBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_vacuumOnBehaviour_fun1(self): 
    self.log("[Behaviour vacuumOnBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - vacuumOnBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_vacuumOnBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_vacuumOnBehaviour_fun1_0(self): 
    self.log("[Behaviour vacuumOnBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - vacuumOnBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- vacuumOnBehaviour")
    self.desiredRobotCommandTele.data="vacuum turn on"
    self.taskStatus.data="task status: vacuum turn on"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_vacuumOnBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour vacuumOnBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - vacuumOnBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_vacuumOnBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_vacuumOnBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour vacuumOnBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - vacuumOnBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_vacuumOnBehaviour(self):
    self.log("[Behaviour vacuumOnBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_vacuumOnBehaviour(self):
    self.log("[Behaviour vacuumOnBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour vacuumOnBehaviour #####
  def executeBehaviour_vacuumOnBehaviour(self):
    self.log("[Behaviour vacuumOnBehaviour] -- Executing vacuumOnBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour vacuumOnBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_vacuumOnBehaviour()
      # Sends data! #
      self.sendData_vacuumOnBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_vacuumOnBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_vacuumOnBehaviour() or self.errorCondition_vacuumOnBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour vacuumOffBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_vacuumOffBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour vacuumOffBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_vacuumOffBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour vacuumOffBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_vacuumOffBehaviour(self): 
    self.log("[Behaviour vacuumOffBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - vacuumOffBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_vacuumOffBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_vacuumOffBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_vacuumOffBehaviour_fun1(self): 
    self.log("[Behaviour vacuumOffBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - vacuumOffBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_vacuumOffBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_vacuumOffBehaviour_fun1_0(self): 
    self.log("[Behaviour vacuumOffBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - vacuumOffBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- vacuumOffBehaviour")
    self.desiredRobotCommandTele.data="vacuum off"
    self.taskStatus.data="task status: vacuum off"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_vacuumOffBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour vacuumOffBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - vacuumOffBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_vacuumOffBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_vacuumOffBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour vacuumOffBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - vacuumOffBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_vacuumOffBehaviour(self):
    self.log("[Behaviour vacuumOffBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_vacuumOffBehaviour(self):
    self.log("[Behaviour vacuumOffBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour vacuumOffBehaviour #####
  def executeBehaviour_vacuumOffBehaviour(self):
    self.log("[Behaviour vacuumOffBehaviour] -- Executing vacuumOffBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour vacuumOffBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_vacuumOffBehaviour()
      # Sends data! #
      self.sendData_vacuumOffBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_vacuumOffBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_vacuumOffBehaviour() or self.errorCondition_vacuumOffBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveBackwardBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveBackwardBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveBackwardBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveBackwardBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveBackwardBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveBackwardBehaviour(self): 
    self.log("[Behaviour moveBackwardBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveBackwardBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveBackwardBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveBackwardBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveBackwardBehaviour_fun1(self): 
    self.log("[Behaviour moveBackwardBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveBackwardBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveBackwardBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveBackwardBehaviour_fun1_0(self): 
    self.log("[Behaviour moveBackwardBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveBackwardBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- moveBackwardBehaviour")
    self.desiredRobotCommandTele.data="move backwards"
    self.taskStatus.data="task status: move backwards"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveBackwardBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveBackwardBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveBackwardBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveBackwardBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveBackwardBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveBackwardBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveBackwardBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveBackwardBehaviour(self):
    self.log("[Behaviour moveBackwardBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveBackwardBehaviour(self):
    self.log("[Behaviour moveBackwardBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveBackwardBehaviour #####
  def executeBehaviour_moveBackwardBehaviour(self):
    self.log("[Behaviour moveBackwardBehaviour] -- Executing moveBackwardBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveBackwardBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveBackwardBehaviour()
      # Sends data! #
      self.sendData_moveBackwardBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveBackwardBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveBackwardBehaviour() or self.errorCondition_moveBackwardBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveFrontBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveFrontBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveFrontBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveFrontBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveFrontBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveFrontBehaviour(self): 
    self.log("[Behaviour moveFrontBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveFrontBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveFrontBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveFrontBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveFrontBehaviour_fun1(self): 
    self.log("[Behaviour moveFrontBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveFrontBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveFrontBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveFrontBehaviour_fun1_0(self): 
    self.log("[Behaviour moveFrontBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveFrontBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- moveFrontBehaviour")
    self.desiredRobotCommandTele.data="move front"
    self.taskStatus.data="task status: move front"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveFrontBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveFrontBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveFrontBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveFrontBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveFrontBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveFrontBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveFrontBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveFrontBehaviour(self):
    self.log("[Behaviour moveFrontBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveFrontBehaviour(self):
    self.log("[Behaviour moveFrontBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveFrontBehaviour #####
  def executeBehaviour_moveFrontBehaviour(self):
    self.log("[Behaviour moveFrontBehaviour] -- Executing moveFrontBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveFrontBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveFrontBehaviour()
      # Sends data! #
      self.sendData_moveFrontBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveFrontBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveFrontBehaviour() or self.errorCondition_moveFrontBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveRightBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveRightBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveRightBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveRightBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveRightBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveRightBehaviour(self): 
    self.log("[Behaviour moveRightBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveRightBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveRightBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveRightBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveRightBehaviour_fun1(self): 
    self.log("[Behaviour moveRightBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveRightBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveRightBehaviour_fun1_0(self): 
    self.log("[Behaviour moveRightBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredRobotCommandTele=True
    self._out_flag_taskStatus=True
    print("[CS - teleoperated] -- moveRightBehaviour")
    self.desiredRobotCommandTele.data="move right"
    self.taskStatus.data="task status: move right"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveRightBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveRightBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveRightBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveRightBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveRightBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveRightBehaviour(self):
    self.log("[Behaviour moveRightBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveRightBehaviour(self):
    self.log("[Behaviour moveRightBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveRightBehaviour #####
  def executeBehaviour_moveRightBehaviour(self):
    self.log("[Behaviour moveRightBehaviour] -- Executing moveRightBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveRightBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveRightBehaviour()
      # Sends data! #
      self.sendData_moveRightBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveRightBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveRightBehaviour() or self.errorCondition_moveRightBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour moveLeftBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_moveLeftBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveLeftBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_moveLeftBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour moveLeftBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_moveLeftBehaviour(self): 
    self.log("[Behaviour moveLeftBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - moveLeftBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_moveLeftBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_moveLeftBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveLeftBehaviour_fun1(self): 
    self.log("[Behaviour moveLeftBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveLeftBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_moveLeftBehaviour_fun1_0(self): 
    self.log("[Behaviour moveLeftBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- moveLeftBehaviour")
    self.desiredRobotCommandTele.data="move left"
    self.taskStatus.data="task status: move left"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_moveLeftBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour moveLeftBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - moveLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_moveLeftBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_moveLeftBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour moveLeftBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - moveLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_moveLeftBehaviour(self):
    self.log("[Behaviour moveLeftBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_moveLeftBehaviour(self):
    self.log("[Behaviour moveLeftBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour moveLeftBehaviour #####
  def executeBehaviour_moveLeftBehaviour(self):
    self.log("[Behaviour moveLeftBehaviour] -- Executing moveLeftBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour moveLeftBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_moveLeftBehaviour()
      # Sends data! #
      self.sendData_moveLeftBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_moveLeftBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_moveLeftBehaviour() or self.errorCondition_moveLeftBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour rotateLeftBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_rotateLeftBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour rotateLeftBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_rotateLeftBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour rotateLeftBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_rotateLeftBehaviour(self): 
    self.log("[Behaviour rotateLeftBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - rotateLeftBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_rotateLeftBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_rotateLeftBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_rotateLeftBehaviour_fun1(self): 
    self.log("[Behaviour rotateLeftBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - rotateLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_rotateLeftBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_rotateLeftBehaviour_fun1_0(self): 
    self.log("[Behaviour rotateLeftBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - rotateLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- rotateLeftBehaviour")
    self.desiredRobotCommandTele.data="rotate left"
    self.taskStatus.data="task status: rotate left"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_rotateLeftBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour rotateLeftBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - rotateLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_rotateLeftBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_rotateLeftBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour rotateLeftBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - rotateLeftBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_rotateLeftBehaviour(self):
    self.log("[Behaviour rotateLeftBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_rotateLeftBehaviour(self):
    self.log("[Behaviour rotateLeftBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour rotateLeftBehaviour #####
  def executeBehaviour_rotateLeftBehaviour(self):
    self.log("[Behaviour rotateLeftBehaviour] -- Executing rotateLeftBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour rotateLeftBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_rotateLeftBehaviour()
      # Sends data! #
      self.sendData_rotateLeftBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_rotateLeftBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_rotateLeftBehaviour() or self.errorCondition_rotateLeftBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour rotateRightBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_rotateRightBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour rotateRightBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_rotateRightBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour rotateRightBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_rotateRightBehaviour(self): 
    self.log("[Behaviour rotateRightBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - rotateRightBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_rotateRightBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_rotateRightBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_rotateRightBehaviour_fun1(self): 
    self.log("[Behaviour rotateRightBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - rotateRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_rotateRightBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_rotateRightBehaviour_fun1_0(self): 
    self.log("[Behaviour rotateRightBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - rotateRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- rotateRightBehaviour")
    self.desiredRobotCommandTele.data="rotate right"
    self.taskStatus.data="task status: rotate right"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_rotateRightBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour rotateRightBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - rotateRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_rotateRightBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_rotateRightBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour rotateRightBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - rotateRightBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_rotateRightBehaviour(self):
    self.log("[Behaviour rotateRightBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_rotateRightBehaviour(self):
    self.log("[Behaviour rotateRightBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour rotateRightBehaviour #####
  def executeBehaviour_rotateRightBehaviour(self):
    self.log("[Behaviour rotateRightBehaviour] -- Executing rotateRightBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour rotateRightBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_rotateRightBehaviour()
      # Sends data! #
      self.sendData_rotateRightBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_rotateRightBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_rotateRightBehaviour() or self.errorCondition_rotateRightBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour whatCanYouSeeBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_whatCanYouSeeBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_whatCanYouSeeBehaviour(self): # String taskCommand, String robotStatus, CameraMessage ballInfoTele, ObstacleDetected obstacleDetectedTele, Bool ballCollected, Int64 isNewCommandReceived, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballInfoTele, std_msgs::Bool _in_flag_obstacleDetectedTele, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotCommandTele #
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_whatCanYouSeeBehaviour(self): 
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - whatCanYouSeeBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_whatCanYouSeeBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_whatCanYouSeeBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_whatCanYouSeeBehaviour_fun1(self): 
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - whatCanYouSeeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_whatCanYouSeeBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_whatCanYouSeeBehaviour_fun1_0(self): 
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - whatCanYouSeeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandTele=True
    print("[CS - teleoperated] -- whatCanYouSeeBehaviour")
    self.desiredRobotCommandTele.data="what can you see"
    self.taskCommand.data="empty"
    self.taskStatus.data="task status: what can you see"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_whatCanYouSeeBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - whatCanYouSeeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_whatCanYouSeeBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_whatCanYouSeeBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - whatCanYouSeeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_whatCanYouSeeBehaviour(self):
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotCommandTele has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandTele ):
      # send data from output buffer desiredRobotCommandTele
      # Buffer desiredRobotCommandTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandTele.publish(self.desiredRobotCommandTele) # sending data from output buffer desiredRobotCommandTele #
      # indicate that data was sent and now the output buffer desiredRobotCommandTele is empty
      self._out_flag_desiredRobotCommandTele=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_whatCanYouSeeBehaviour(self):
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer ballInfoTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballInfoTele
    # Buffer obstacleDetectedTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedTele
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour whatCanYouSeeBehaviour #####
  def executeBehaviour_whatCanYouSeeBehaviour(self):
    self.log("[Behaviour whatCanYouSeeBehaviour] -- Executing whatCanYouSeeBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour whatCanYouSeeBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_whatCanYouSeeBehaviour()
      # Sends data! #
      self.sendData_whatCanYouSeeBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_whatCanYouSeeBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_whatCanYouSeeBehaviour() or self.errorCondition_whatCanYouSeeBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass


  ##### Definition of functions responsible for switching subsystem cs between states : Behaviour_initBehaviour Behaviour_idleBehaviour Behaviour_terminateBehaviour Behaviour_terminatedBehaviour Behaviour_stopBehaviour Behaviour_moveFasterBehaviour Behaviour_moveSlowerBehaviour Behaviour_vacuumOnBehaviour Behaviour_vacuumOffBehaviour Behaviour_moveBackwardBehaviour Behaviour_moveFrontBehaviour Behaviour_moveRightBehaviour Behaviour_moveLeftBehaviour Behaviour_rotateLeftBehaviour Behaviour_rotateRightBehaviour Behaviour_whatCanYouSeeBehaviour  #####
  # Behaviour initBehaviour: #
  def subsystemBehaviour_initBehaviour(self):
    self.log("subsystemBehaviour_initBehaviour")
    # Executing behaviour initBehaviour #
    self.executeBehaviour_initBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour initBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_initBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour idleBehaviour: #
  def subsystemBehaviour_idleBehaviour(self):
    self.log("subsystemBehaviour_idleBehaviour")
    # Executing behaviour idleBehaviour #
    self.executeBehaviour_idleBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour idleBehaviour: switching to behaviour terminateBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_terminateBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminateBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour stopBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_stopBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_stopBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveFasterBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveFasterBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveFasterBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveSlowerBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveSlowerBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveSlowerBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour vacuumOnBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_vacuumOnBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_vacuumOnBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour vacuumOffBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_vacuumOffBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_vacuumOffBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveBackwardBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveBackwardBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveBackwardBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveFrontBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveFrontBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveFrontBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveRightBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveRightBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveRightBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour moveLeftBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveLeftBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_moveLeftBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour rotateLeftBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_rotateLeftBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_rotateLeftBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour rotateRightBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_rotateRightBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_rotateRightBehaviour"

    # Checking initial condition for behaviour idleBehaviour: switching to behaviour whatCanYouSeeBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_whatCanYouSeeBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_whatCanYouSeeBehaviour"

    pass


  # Behaviour terminateBehaviour: #
  def subsystemBehaviour_terminateBehaviour(self):
    self.log("subsystemBehaviour_terminateBehaviour")
    # Executing behaviour terminateBehaviour #
    self.executeBehaviour_terminateBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour terminateBehaviour: switching to behaviour terminatedBehaviour #
    if self.initialCondition_From_Behaviour_terminateBehaviour_To_Behaviour_terminatedBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminatedBehaviour"

    pass


  # Behaviour terminatedBehaviour: #
  def subsystemBehaviour_terminatedBehaviour(self):
    self.log("subsystemBehaviour_terminatedBehaviour")
    # Executing behaviour terminatedBehaviour #
    self.executeBehaviour_terminatedBehaviour()
    # Behaviour has been terminated #
    pass


  # Behaviour stopBehaviour: #
  def subsystemBehaviour_stopBehaviour(self):
    self.log("subsystemBehaviour_stopBehaviour")
    # Executing behaviour stopBehaviour #
    self.executeBehaviour_stopBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour stopBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_stopBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveFasterBehaviour: #
  def subsystemBehaviour_moveFasterBehaviour(self):
    self.log("subsystemBehaviour_moveFasterBehaviour")
    # Executing behaviour moveFasterBehaviour #
    self.executeBehaviour_moveFasterBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveFasterBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveFasterBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveSlowerBehaviour: #
  def subsystemBehaviour_moveSlowerBehaviour(self):
    self.log("subsystemBehaviour_moveSlowerBehaviour")
    # Executing behaviour moveSlowerBehaviour #
    self.executeBehaviour_moveSlowerBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveSlowerBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveSlowerBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour vacuumOnBehaviour: #
  def subsystemBehaviour_vacuumOnBehaviour(self):
    self.log("subsystemBehaviour_vacuumOnBehaviour")
    # Executing behaviour vacuumOnBehaviour #
    self.executeBehaviour_vacuumOnBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour vacuumOnBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_vacuumOnBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour vacuumOffBehaviour: #
  def subsystemBehaviour_vacuumOffBehaviour(self):
    self.log("subsystemBehaviour_vacuumOffBehaviour")
    # Executing behaviour vacuumOffBehaviour #
    self.executeBehaviour_vacuumOffBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour vacuumOffBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_vacuumOffBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveBackwardBehaviour: #
  def subsystemBehaviour_moveBackwardBehaviour(self):
    self.log("subsystemBehaviour_moveBackwardBehaviour")
    # Executing behaviour moveBackwardBehaviour #
    self.executeBehaviour_moveBackwardBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveBackwardBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveBackwardBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveFrontBehaviour: #
  def subsystemBehaviour_moveFrontBehaviour(self):
    self.log("subsystemBehaviour_moveFrontBehaviour")
    # Executing behaviour moveFrontBehaviour #
    self.executeBehaviour_moveFrontBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveFrontBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveFrontBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveRightBehaviour: #
  def subsystemBehaviour_moveRightBehaviour(self):
    self.log("subsystemBehaviour_moveRightBehaviour")
    # Executing behaviour moveRightBehaviour #
    self.executeBehaviour_moveRightBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveRightBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveRightBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour moveLeftBehaviour: #
  def subsystemBehaviour_moveLeftBehaviour(self):
    self.log("subsystemBehaviour_moveLeftBehaviour")
    # Executing behaviour moveLeftBehaviour #
    self.executeBehaviour_moveLeftBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour moveLeftBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_moveLeftBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour rotateLeftBehaviour: #
  def subsystemBehaviour_rotateLeftBehaviour(self):
    self.log("subsystemBehaviour_rotateLeftBehaviour")
    # Executing behaviour rotateLeftBehaviour #
    self.executeBehaviour_rotateLeftBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour rotateLeftBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_rotateLeftBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour rotateRightBehaviour: #
  def subsystemBehaviour_rotateRightBehaviour(self):
    self.log("subsystemBehaviour_rotateRightBehaviour")
    # Executing behaviour rotateRightBehaviour #
    self.executeBehaviour_rotateRightBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour rotateRightBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_rotateRightBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  # Behaviour whatCanYouSeeBehaviour: #
  def subsystemBehaviour_whatCanYouSeeBehaviour(self):
    self.log("subsystemBehaviour_whatCanYouSeeBehaviour")
    # Executing behaviour whatCanYouSeeBehaviour #
    self.executeBehaviour_whatCanYouSeeBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour whatCanYouSeeBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_whatCanYouSeeBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  ##### Initial condition for behaviour initBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_initBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour terminateBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_terminateBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="terminate" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour stopBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_stopBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="stop" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveFasterBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveFasterBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="speed up" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveSlowerBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveSlowerBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="speed down" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour vacuumOnBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_vacuumOnBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="vacuum turn on" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour vacuumOffBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_vacuumOffBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="vacuum off" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveBackwardBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveBackwardBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="backwards" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveFrontBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveFrontBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="front" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveRightBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveRightBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="right" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour moveLeftBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_moveLeftBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="left" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour rotateLeftBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_rotateLeftBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="rotate left" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour rotateRightBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_rotateRightBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="rotate right" 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour whatCanYouSeeBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_whatCanYouSeeBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="what can you see" 


  ##### Initial condition for behaviour terminateBehaviour: switching to behaviour terminatedBehaviour #####
  def initialCondition_From_Behaviour_terminateBehaviour_To_Behaviour_terminatedBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour stopBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_stopBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveFasterBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveFasterBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveSlowerBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveSlowerBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour vacuumOnBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_vacuumOnBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour vacuumOffBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_vacuumOffBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveBackwardBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveBackwardBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveFrontBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveFrontBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveRightBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveRightBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour moveLeftBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_moveLeftBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour rotateLeftBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_rotateLeftBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour rotateRightBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_rotateRightBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour whatCanYouSeeBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_whatCanYouSeeBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 



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

