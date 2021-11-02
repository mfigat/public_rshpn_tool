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
from auxiliary_agent_audio import *
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
    self._subsystemFrequency=4;
    self._currentSubsystemBehaviour="Behaviour_initBehaviour";
    self._subsystemIterations=0
    self._behaviourIterations=0
    self.initialiseCommunicationModel()
    self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
    # initialize all input flags
    self._in_flag_veCSRecognisedCommand=False
    self._in_flag_statusCommand=False
    # initialize all output flags
    self._out_flag_textToSynthesize=False
    self._out_flag_recognisedCommand=False
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
    except Exception as e:
      print e
      self.error("Error found in function startSubsystem -- file subsystem_cs.py!")
      pass

  ##### Update data for input buffer: veCSRecognisedCommand #####
  def update_veCSRecognisedCommand(self, data):
    self.log("update_veCSRecognisedCommand")
    self.veCSRecognisedCommand=data
    self._in_flag_veCSRecognisedCommand=True
    pass

  ##### Update data for input buffer: statusCommand #####
  def update_statusCommand(self, data):
    self.log("update_statusCommand")
    self.statusCommand=data
    self._in_flag_statusCommand=True
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
    # Buffer name=textToSynthesize - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_textToSynthesize=rospy.Publisher("textToSynthesize", String, queue_size=CHANNEL_SIZE)
    # Buffer name=recognisedCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_recognisedCommand=rospy.Publisher("recognisedCommandChannel", String, queue_size=CHANNEL_SIZE)
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
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_veCSRecognisedCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_statusCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_textToSynthesize', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_recognisedCommand', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=veCSRecognisedCommand sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_veCSRecognisedCommand=rospy.Subscriber("veCSRecognisedCommand", String, self.update_veCSRecognisedCommand)
    # Buffer name=statusCommand sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_statusCommand=rospy.Subscriber("statusCommandChannel", String, self.update_statusCommand)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
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
    if(8 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self._in_flag_veCSRecognisedCommand)
      self._vectorOfSenderDiagnostics[6].publish(self._in_flag_statusCommand)
      self._vectorOfSenderDiagnostics[7].publish(self._out_flag_textToSynthesize)
      self._vectorOfSenderDiagnostics[8].publish(self._out_flag_recognisedCommand)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # String veCSRecognisedCommand, String statusCommand, std_msgs::Bool _in_flag_veCSRecognisedCommand, std_msgs::Bool _in_flag_statusCommand, std_msgs::Bool _out_flag_textToSynthesize, std_msgs::Bool _out_flag_recognisedCommand #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # String veCSRecognisedCommand, String statusCommand, std_msgs::Bool _in_flag_veCSRecognisedCommand, std_msgs::Bool _in_flag_statusCommand, std_msgs::Bool _out_flag_textToSynthesize, std_msgs::Bool _out_flag_recognisedCommand #
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
    print("[CS - audio] -- initBehaviour")
    self.textToSynthesize=String("empty")
    self.recognisedCommand=String("empty")
    self.veCSRecognisedCommand=String("empty")
    self.statusCommand=String("empty")
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
    # check if output buffer textToSynthesize has new data - i.e. is ready to send new data
    if( self._out_flag_textToSynthesize ):
      # send data from output buffer textToSynthesize
      # Buffer textToSynthesize - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_textToSynthesize.publish(self.textToSynthesize) # sending data from output buffer textToSynthesize #
      # indicate that data was sent and now the output buffer textToSynthesize is empty
      self._out_flag_textToSynthesize=False
    # check if output buffer recognisedCommand has new data - i.e. is ready to send new data
    if( self._out_flag_recognisedCommand ):
      # send data from output buffer recognisedCommand
      # Buffer recognisedCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_recognisedCommand.publish(self.recognisedCommand) # sending data from output buffer recognisedCommand #
      # indicate that data was sent and now the output buffer recognisedCommand is empty
      self._out_flag_recognisedCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer veCSRecognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer veCSRecognisedCommand
    # Buffer statusCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer statusCommand
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
  def terminalCondition_idleBehaviour(self): # String veCSRecognisedCommand, String statusCommand, std_msgs::Bool _in_flag_veCSRecognisedCommand, std_msgs::Bool _in_flag_statusCommand, std_msgs::Bool _out_flag_textToSynthesize, std_msgs::Bool _out_flag_recognisedCommand #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # String veCSRecognisedCommand, String statusCommand, std_msgs::Bool _in_flag_veCSRecognisedCommand, std_msgs::Bool _in_flag_statusCommand, std_msgs::Bool _out_flag_textToSynthesize, std_msgs::Bool _out_flag_recognisedCommand #
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
    # Partial transition function call: fun2
    self.transitionFunction_idleBehaviour_fun2()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_idleBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand:
      self.transitionFunction_idleBehaviour_fun1_0()
    elif self._in_flag_veCSRecognisedCommand and  not self._in_flag_statusCommand:
      self.transitionFunction_idleBehaviour_fun1_1()
    elif  not self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand:
      self.transitionFunction_idleBehaviour_fun1_2()
    elif  not (self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand) and  not (self._in_flag_veCSRecognisedCommand and  not self._in_flag_statusCommand) and  not ( not self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand):
      self.transitionFunction_idleBehaviour_fun1_3()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand #####
  def transitionFunction_idleBehaviour_fun1_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_textToSynthesize=True
    if(checkCommand(self.veCSRecognisedCommand.data)):
      self.textToSynthesize.data="Command: "+self.veCSRecognisedCommand.data + self.statusCommand.data
    else:
      self._out_flag_textToSynthesize=False
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers self._in_flag_veCSRecognisedCommand and  not self._in_flag_statusCommand #####
  def transitionFunction_idleBehaviour_fun1_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_textToSynthesize=True
    if(checkCommand(self.veCSRecognisedCommand.data)):
      self.textToSynthesize.data="Command: "+self.veCSRecognisedCommand.data
    else:
      self._out_flag_textToSynthesize=False
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_2 based on input buffers  not self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand #####
  def transitionFunction_idleBehaviour_fun1_2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_textToSynthesize=True
    if(self.statusCommand.data!="empty"):
      self.textToSynthesize.data=self.statusCommand.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_3 based on input buffers  not (self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand) and  not (self._in_flag_veCSRecognisedCommand and  not self._in_flag_statusCommand) and  not ( not self._in_flag_veCSRecognisedCommand and self._in_flag_statusCommand) #####
  def transitionFunction_idleBehaviour_fun1_3(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_3")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - audio] -- textToSynthesize")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_veCSRecognisedCommand:
      self.transitionFunction_idleBehaviour_fun2_0()
    elif  not (self._in_flag_veCSRecognisedCommand):
      self.transitionFunction_idleBehaviour_fun2_1()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_veCSRecognisedCommand #####
  def transitionFunction_idleBehaviour_fun2_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_recognisedCommand=True
    print("[CS - audio] -- recognisedCommand")
    if(checkCommand(self.veCSRecognisedCommand.data)):
      self.recognisedCommand.data=self.veCSRecognisedCommand.data
    else:
      self._out_flag_recognisedCommand=False 
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not (self._in_flag_veCSRecognisedCommand) #####
  def transitionFunction_idleBehaviour_fun2_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
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
    self._in_flag_veCSRecognisedCommand=False
    self._in_flag_statusCommand=False
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
    # check if output buffer textToSynthesize has new data - i.e. is ready to send new data
    if( self._out_flag_textToSynthesize ):
      # send data from output buffer textToSynthesize
      # Buffer textToSynthesize - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_textToSynthesize.publish(self.textToSynthesize) # sending data from output buffer textToSynthesize #
      # indicate that data was sent and now the output buffer textToSynthesize is empty
      self._out_flag_textToSynthesize=False
    # check if output buffer recognisedCommand has new data - i.e. is ready to send new data
    if( self._out_flag_recognisedCommand ):
      # send data from output buffer recognisedCommand
      # Buffer recognisedCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_recognisedCommand.publish(self.recognisedCommand) # sending data from output buffer recognisedCommand #
      # indicate that data was sent and now the output buffer recognisedCommand is empty
      self._out_flag_recognisedCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer veCSRecognisedCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer veCSRecognisedCommand
    # Buffer statusCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer statusCommand
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


  ##### Definition of functions responsible for switching subsystem cs between states : Behaviour_initBehaviour Behaviour_idleBehaviour  #####
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
    pass


  ##### Initial condition for behaviour initBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_initBehaviour_To_Behaviour_idleBehaviour(self):
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

