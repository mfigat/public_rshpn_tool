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
from auxiliary_agent_supervisor import *
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
    self._in_flag_recognisedCommand=False
    self._in_flag_taskStatusTele=False
    self._in_flag_taskStatusAuto=False
    # initialize all output flags
    self._out_flag_statusCommand=False
    self._out_flag_taskCommand=False
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
        if self._currentSubsystemBehaviour=="Behaviour_taskActivationBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_taskActivationBehaviour")
          self.subsystemBehaviour_taskActivationBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_autonomousActivatedBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_autonomousActivatedBehaviour")
          self.subsystemBehaviour_autonomousActivatedBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_teleoperatedActivatedBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_teleoperatedActivatedBehaviour")
          self.subsystemBehaviour_teleoperatedActivatedBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_stopRobotBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_stopRobotBehaviour")
          self.subsystemBehaviour_stopRobotBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_terminateBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_terminateBehaviour")
          self.subsystemBehaviour_terminateBehaviour()
          continue
    except Exception as e:
      print e
      self.error("Error found in function startSubsystem -- file subsystem_cs.py!")
      pass

  ##### Update data for input buffer: recognisedCommand #####
  def update_recognisedCommand(self, data):
    self.log("update_recognisedCommand")
    self.recognisedCommand=data
    self._in_flag_recognisedCommand=True
    pass

  ##### Update data for input buffer: taskStatusTele #####
  def update_taskStatusTele(self, data):
    self.log("update_taskStatusTele")
    self.taskStatusTele=data
    self._in_flag_taskStatusTele=True
    pass

  ##### Update data for input buffer: taskStatusAuto #####
  def update_taskStatusAuto(self, data):
    self.log("update_taskStatusAuto")
    self.taskStatusAuto=data
    self._in_flag_taskStatusAuto=True
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
    # Buffer name=statusCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_statusCommand=rospy.Publisher("statusCommandChannel", String, queue_size=CHANNEL_SIZE)
    # Buffer name=taskCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_taskCommand=rospy.Publisher("taskCommandChannel", String, queue_size=CHANNEL_SIZE)
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
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/isTaskActivated', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/commandReceived', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/taskToTerminate', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_recognisedCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_taskStatusTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_taskStatusAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_statusCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_taskCommand', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=recognisedCommand sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_recognisedCommand=rospy.Subscriber("recognisedCommandChannel", String, self.update_recognisedCommand)
    # Buffer name=taskStatusTele sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_taskStatusTele=rospy.Subscriber("taskStatusChannelTele", String, self.update_taskStatusTele)
    # Buffer name=taskStatusAuto sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_taskStatusAuto=rospy.Subscriber("taskStatusChannelAuto", String, self.update_taskStatusAuto)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
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
      self._vectorOfSenderDiagnostics[5].publish(self.isTaskActivated)
      self._vectorOfSenderDiagnostics[6].publish(self.commandReceived)
      self._vectorOfSenderDiagnostics[7].publish(self.taskToTerminate)
      self._vectorOfSenderDiagnostics[8].publish(self._in_flag_recognisedCommand)
      self._vectorOfSenderDiagnostics[9].publish(self._in_flag_taskStatusTele)
      self._vectorOfSenderDiagnostics[10].publish(self._in_flag_taskStatusAuto)
      self._vectorOfSenderDiagnostics[11].publish(self._out_flag_statusCommand)
      self._vectorOfSenderDiagnostics[12].publish(self._out_flag_taskCommand)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
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
    print("[CS - supervisor] -- initBehaviour")
    self.recognisedCommand=String("empty")
    self.statusCommand=String("empty")
    self.isTaskActivated=Bool(False)
    self.commandReceived=String("empty")
    self.taskCommand=String("empty")
    self.taskToTerminate=Bool(False)
    self.taskStatusAuto=String("empty")
    self.taskStatusTele=String("empty")
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
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
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
  def terminalCondition_idleBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  self.recognisedCommand.data == "activate teleoperated" or self.recognisedCommand.data == "activate autonomous" 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
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
    if True:
      self.transitionFunction_idleBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_idleBehaviour_fun1_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - supervisor] -- idleBehaviour")
    self.statusCommand.data="empty"
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
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
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

  ##### Behaviour taskActivationBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_taskActivationBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour taskActivationBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_taskActivationBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour taskActivationBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_taskActivationBehaviour(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - taskActivationBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_taskActivationBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_taskActivationBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_taskActivationBehaviour_fun1(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - taskActivationBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_recognisedCommand:
      self.transitionFunction_taskActivationBehaviour_fun1_0()
    elif  not (self._in_flag_recognisedCommand):
      self.transitionFunction_taskActivationBehaviour_fun1_1()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_recognisedCommand #####
  def transitionFunction_taskActivationBehaviour_fun1_0(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - taskActivationBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_statusCommand=True
    global uuid
    global launch
    print("[CS - supervisor] -- taskActivationBehaviour")
    flag=False
    launch_file_path=""
    if(self.recognisedCommand.data=="activate teleoperated"):
      self.statusCommand.data="Teleoperated task activated"
      flag=True
      launch_file_path=teleoperated_agent_launch_file
      self.commandReceived.data="activate teleoperated"
    elif (self.recognisedCommand.data=="activate autonomous"):
      self.statusCommand.data="Autonomous task activated"
      flag=True
      launch_file_path=autonomous_agent_launch_file
      self.commandReceived.data="activate autonomous"
    else:
      self.statusCommand.data="empty"
    if(flag):
      self.isTaskActivated.data=True
      uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
      roslaunch.configure_logging(uuid)
      launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
      # activate launch file
      launch.start()
      rospy.loginfo("launch file="+launch_file_path+" -- activated")
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not (self._in_flag_recognisedCommand) #####
  def transitionFunction_taskActivationBehaviour_fun1_1(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - taskActivationBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_statusCommand=True
    print("[CS - supervisor] -- taskActivationBehaviour")
    self.statusCommand=String("empty")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_taskActivationBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - taskActivationBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_taskActivationBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_taskActivationBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour taskActivationBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - taskActivationBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_recognisedCommand=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_taskActivationBehaviour(self):
    self.log("[Behaviour taskActivationBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_taskActivationBehaviour(self):
    self.log("[Behaviour taskActivationBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour taskActivationBehaviour #####
  def executeBehaviour_taskActivationBehaviour(self):
    self.log("[Behaviour taskActivationBehaviour] -- Executing taskActivationBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour taskActivationBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_taskActivationBehaviour()
      # Sends data! #
      self.sendData_taskActivationBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_taskActivationBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_taskActivationBehaviour() or self.errorCondition_taskActivationBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour autonomousActivatedBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_autonomousActivatedBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour autonomousActivatedBehaviour] -- Checking Terminal Condition")
    return  self.recognisedCommand.data == "terminate" 
    pass

  ##### Error condition #####
  def errorCondition_autonomousActivatedBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour autonomousActivatedBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_autonomousActivatedBehaviour(self): 
    self.log("[Behaviour autonomousActivatedBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - autonomousActivatedBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_autonomousActivatedBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_autonomousActivatedBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_autonomousActivatedBehaviour_fun1(self): 
    self.log("[Behaviour autonomousActivatedBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - autonomousActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_autonomousActivatedBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_autonomousActivatedBehaviour_fun1_0(self): 
    self.log("[Behaviour autonomousActivatedBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - autonomousActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - supervisor] -- autonomousActivatedBehaviour")
    self.statusCommand=String("empty")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_autonomousActivatedBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour autonomousActivatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - autonomousActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_autonomousActivatedBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_autonomousActivatedBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour autonomousActivatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - autonomousActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_autonomousActivatedBehaviour(self):
    self.log("[Behaviour autonomousActivatedBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_autonomousActivatedBehaviour(self):
    self.log("[Behaviour autonomousActivatedBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour autonomousActivatedBehaviour #####
  def executeBehaviour_autonomousActivatedBehaviour(self):
    self.log("[Behaviour autonomousActivatedBehaviour] -- Executing autonomousActivatedBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour autonomousActivatedBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_autonomousActivatedBehaviour()
      # Sends data! #
      self.sendData_autonomousActivatedBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_autonomousActivatedBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_autonomousActivatedBehaviour() or self.errorCondition_autonomousActivatedBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour teleoperatedActivatedBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_teleoperatedActivatedBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Checking Terminal Condition")
    return  self.recognisedCommand.data == "terminate" 
    pass

  ##### Error condition #####
  def errorCondition_teleoperatedActivatedBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_teleoperatedActivatedBehaviour(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_teleoperatedActivatedBehaviour_fun1()
    # Partial transition function call: fun2
    self.transitionFunction_teleoperatedActivatedBehaviour_fun2()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_teleoperatedActivatedBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun1(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_recognisedCommand:
      self.transitionFunction_teleoperatedActivatedBehaviour_fun1_0()
    elif  not (self._in_flag_recognisedCommand):
      self.transitionFunction_teleoperatedActivatedBehaviour_fun1_1()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_recognisedCommand #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun1_0(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskCommand=True
    print("[CS - supervisor] -- teleoperatedActivatedBehaviour")
    if(self.recognisedCommand.data!="terminate"):
      self.taskCommand.data=self.recognisedCommand.data
    else:
      self._out_flag_taskCommand=False
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not (self._in_flag_recognisedCommand) #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun1_1(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskCommand=True
    print("[CS - supervisor] -- teleoperatedActivatedBehaviour")
    self._out_flag_taskCommand=False
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun2(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_taskStatusTele and  not self._in_flag_taskStatusAuto:
      self.transitionFunction_teleoperatedActivatedBehaviour_fun2_0()
    elif  not self._in_flag_taskStatusTele and self._in_flag_taskStatusAuto:
      self.transitionFunction_teleoperatedActivatedBehaviour_fun2_1()
    elif  not (self._in_flag_taskStatusTele and  not self._in_flag_taskStatusAuto) and  not ( not self._in_flag_taskStatusTele and self._in_flag_taskStatusAuto):
      self.transitionFunction_teleoperatedActivatedBehaviour_fun2_2()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_taskStatusTele and  not self._in_flag_taskStatusAuto #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun2_0(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_statusCommand=True
    print("[CS - supervisor] -- teleoperatedActivatedBehaviour")
    self.statusCommand.data=self.taskStatusTele.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not self._in_flag_taskStatusTele and self._in_flag_taskStatusAuto #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun2_1(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_statusCommand=True
    print("[CS - supervisor] -- teleoperatedActivatedBehaviour")
    self.statusCommand.data=self.taskStatusAuto.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_2 based on input buffers  not (self._in_flag_taskStatusTele and  not self._in_flag_taskStatusAuto) and  not ( not self._in_flag_taskStatusTele and self._in_flag_taskStatusAuto) #####
  def transitionFunction_teleoperatedActivatedBehaviour_fun2_2(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function fun2_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_statusCommand=True
    print("[CS - supervisor] -- teleoperatedActivatedBehaviour")
    self._out_flag_statusCommand=False
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_teleoperatedActivatedBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_teleoperatedActivatedBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_teleoperatedActivatedBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - teleoperatedActivatedBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_recognisedCommand=False
    self._in_flag_taskStatusTele=False
    self._in_flag_taskStatusAuto=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_teleoperatedActivatedBehaviour(self):
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_teleoperatedActivatedBehaviour(self):
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour teleoperatedActivatedBehaviour #####
  def executeBehaviour_teleoperatedActivatedBehaviour(self):
    self.log("[Behaviour teleoperatedActivatedBehaviour] -- Executing teleoperatedActivatedBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour teleoperatedActivatedBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_teleoperatedActivatedBehaviour()
      # Sends data! #
      self.sendData_teleoperatedActivatedBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_teleoperatedActivatedBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_teleoperatedActivatedBehaviour() or self.errorCondition_teleoperatedActivatedBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour stopRobotBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_stopRobotBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour stopRobotBehaviour] -- Checking Terminal Condition")
    return  self.taskStatusTele.data == "task terminated" or self.taskStatusAuto.data == "task terminated" 
    pass

  ##### Error condition #####
  def errorCondition_stopRobotBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour stopRobotBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_stopRobotBehaviour(self): 
    self.log("[Behaviour stopRobotBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - stopRobotBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_stopRobotBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_stopRobotBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_stopRobotBehaviour_fun1(self): 
    self.log("[Behaviour stopRobotBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - stopRobotBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_stopRobotBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_stopRobotBehaviour_fun1_0(self): 
    self.log("[Behaviour stopRobotBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - stopRobotBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskCommand=True
    print("[CS - supervisor] -- stopRobotBehaviour")
    if(self.taskCommand.data=="terminate"):
      self._out_flag_taskCommand=False
    else:
      self.taskCommand.data="terminate"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_stopRobotBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour stopRobotBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - stopRobotBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_stopRobotBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_stopRobotBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour stopRobotBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - stopRobotBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_stopRobotBehaviour(self):
    self.log("[Behaviour stopRobotBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_stopRobotBehaviour(self):
    self.log("[Behaviour stopRobotBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour stopRobotBehaviour #####
  def executeBehaviour_stopRobotBehaviour(self):
    self.log("[Behaviour stopRobotBehaviour] -- Executing stopRobotBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour stopRobotBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_stopRobotBehaviour()
      # Sends data! #
      self.sendData_stopRobotBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_stopRobotBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_stopRobotBehaviour() or self.errorCondition_stopRobotBehaviour()
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
  def terminalCondition_terminateBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
    self.log("[Behaviour terminateBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_terminateBehaviour(self): # String recognisedCommand, String taskStatusTele, String taskStatusAuto, Bool isTaskActivated, String commandReceived, Bool taskToTerminate, std_msgs::Bool _in_flag_recognisedCommand, std_msgs::Bool _in_flag_taskStatusTele, std_msgs::Bool _in_flag_taskStatusAuto, std_msgs::Bool _out_flag_statusCommand, std_msgs::Bool _out_flag_taskCommand #
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
    self._out_flag_statusCommand=True
    print("[CS - supervisor] -- terminateBehaviour")
    launch.shutdown() # terminate launch file (if it is activated)
    self.statusCommand.data="Task terminated"
    self.taskStatusTele.data = "empty"
    self.taskStatusAuto.data = "empty"
    self.taskCommand.data="empty"
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
    # check if output buffer statusCommand has new data - i.e. is ready to send new data
    if( self._out_flag_statusCommand ):
      # send data from output buffer statusCommand
      # Buffer statusCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_statusCommand.publish(self.statusCommand) # sending data from output buffer statusCommand #
      # indicate that data was sent and now the output buffer statusCommand is empty
      self._out_flag_statusCommand=False
    # check if output buffer taskCommand has new data - i.e. is ready to send new data
    if( self._out_flag_taskCommand ):
      # send data from output buffer taskCommand
      # Buffer taskCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskCommand.publish(self.taskCommand) # sending data from output buffer taskCommand #
      # indicate that data was sent and now the output buffer taskCommand is empty
      self._out_flag_taskCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_terminateBehaviour(self):
    self.log("[Behaviour terminateBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer recognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer recognisedCommand
    # Buffer taskStatusTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusTele
    # Buffer taskStatusAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskStatusAuto
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


  ##### Definition of functions responsible for switching subsystem cs between states : Behaviour_initBehaviour Behaviour_idleBehaviour Behaviour_taskActivationBehaviour Behaviour_autonomousActivatedBehaviour Behaviour_teleoperatedActivatedBehaviour Behaviour_stopRobotBehaviour Behaviour_terminateBehaviour  #####
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
    # Checking initial condition for behaviour idleBehaviour: switching to behaviour taskActivationBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_taskActivationBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_taskActivationBehaviour"

    pass


  # Behaviour taskActivationBehaviour: #
  def subsystemBehaviour_taskActivationBehaviour(self):
    self.log("subsystemBehaviour_taskActivationBehaviour")
    # Executing behaviour taskActivationBehaviour #
    self.executeBehaviour_taskActivationBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour taskActivationBehaviour: switching to behaviour teleoperatedActivatedBehaviour #
    if self.initialCondition_From_Behaviour_taskActivationBehaviour_To_Behaviour_teleoperatedActivatedBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_teleoperatedActivatedBehaviour"

    # Checking initial condition for behaviour taskActivationBehaviour: switching to behaviour autonomousActivatedBehaviour #
    if self.initialCondition_From_Behaviour_taskActivationBehaviour_To_Behaviour_autonomousActivatedBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_autonomousActivatedBehaviour"

    pass


  # Behaviour autonomousActivatedBehaviour: #
  def subsystemBehaviour_autonomousActivatedBehaviour(self):
    self.log("subsystemBehaviour_autonomousActivatedBehaviour")
    # Executing behaviour autonomousActivatedBehaviour #
    self.executeBehaviour_autonomousActivatedBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour autonomousActivatedBehaviour: switching to behaviour stopRobotBehaviour #
    if self.initialCondition_From_Behaviour_autonomousActivatedBehaviour_To_Behaviour_stopRobotBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_stopRobotBehaviour"

    pass


  # Behaviour teleoperatedActivatedBehaviour: #
  def subsystemBehaviour_teleoperatedActivatedBehaviour(self):
    self.log("subsystemBehaviour_teleoperatedActivatedBehaviour")
    # Executing behaviour teleoperatedActivatedBehaviour #
    self.executeBehaviour_teleoperatedActivatedBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour teleoperatedActivatedBehaviour: switching to behaviour stopRobotBehaviour #
    if self.initialCondition_From_Behaviour_teleoperatedActivatedBehaviour_To_Behaviour_stopRobotBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_stopRobotBehaviour"

    pass


  # Behaviour stopRobotBehaviour: #
  def subsystemBehaviour_stopRobotBehaviour(self):
    self.log("subsystemBehaviour_stopRobotBehaviour")
    # Executing behaviour stopRobotBehaviour #
    self.executeBehaviour_stopRobotBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour stopRobotBehaviour: switching to behaviour terminateBehaviour #
    if self.initialCondition_From_Behaviour_stopRobotBehaviour_To_Behaviour_terminateBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminateBehaviour"

    pass


  # Behaviour terminateBehaviour: #
  def subsystemBehaviour_terminateBehaviour(self):
    self.log("subsystemBehaviour_terminateBehaviour")
    # Executing behaviour terminateBehaviour #
    self.executeBehaviour_terminateBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour terminateBehaviour: switching to behaviour idleBehaviour #
    if self.initialCondition_From_Behaviour_terminateBehaviour_To_Behaviour_idleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviour"

    pass


  ##### Initial condition for behaviour initBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_initBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour taskActivationBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_taskActivationBehaviour(self):
    # Initial condition specified by user #
    return  self.recognisedCommand.data == "activate teleoperated" or self.recognisedCommand.data == "activate autonomous" 


  ##### Initial condition for behaviour taskActivationBehaviour: switching to behaviour teleoperatedActivatedBehaviour #####
  def initialCondition_From_Behaviour_taskActivationBehaviour_To_Behaviour_teleoperatedActivatedBehaviour(self):
    # Initial condition specified by user #
    return  self.commandReceived.data == "activate teleoperated" 


  ##### Initial condition for behaviour taskActivationBehaviour: switching to behaviour autonomousActivatedBehaviour #####
  def initialCondition_From_Behaviour_taskActivationBehaviour_To_Behaviour_autonomousActivatedBehaviour(self):
    # Initial condition specified by user #
    return  self.commandReceived.data == "activate autonomous" 


  ##### Initial condition for behaviour autonomousActivatedBehaviour: switching to behaviour stopRobotBehaviour #####
  def initialCondition_From_Behaviour_autonomousActivatedBehaviour_To_Behaviour_stopRobotBehaviour(self):
    # Initial condition specified by user #
    return  self.recognisedCommand.data == "terminate" 


  ##### Initial condition for behaviour teleoperatedActivatedBehaviour: switching to behaviour stopRobotBehaviour #####
  def initialCondition_From_Behaviour_teleoperatedActivatedBehaviour_To_Behaviour_stopRobotBehaviour(self):
    # Initial condition specified by user #
    return  self.recognisedCommand.data == "terminate" 


  ##### Initial condition for behaviour stopRobotBehaviour: switching to behaviour terminateBehaviour #####
  def initialCondition_From_Behaviour_stopRobotBehaviour_To_Behaviour_terminateBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour terminateBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_terminateBehaviour_To_Behaviour_idleBehaviour(self):
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

