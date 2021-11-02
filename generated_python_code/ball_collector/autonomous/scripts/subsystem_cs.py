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
from auxiliary_agent_autonomous import *
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
    self._subsystemFrequency=5;
    self._currentSubsystemBehaviour="Behaviour_initBehaviour";
    self._subsystemIterations=0
    self._behaviourIterations=0
    self.initialiseCommunicationModel()
    self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
    # initialize all input flags
    self._in_flag_taskCommand=False
    self._in_flag_obstacleDetectedAuto=False
    self._in_flag_robotStatus=False
    self._in_flag_ballCollected=False
    self._in_flag_bestBallDetectedIntel=False
    self._in_flag_bestBallDetectedRpi=False
    # initialize all output flags
    self._out_flag_taskStatus=False
    self._out_flag_desiredRobotSpeedAuto=False
    self._out_flag_desiredRobotCommandAuto=False
    self._out_flag_desiredVaccumSpeedAuto=False
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
        if self._currentSubsystemBehaviour=="Behaviour_findBallBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_findBallBehaviour")
          self.subsystemBehaviour_findBallBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_avoidObstacleBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_avoidObstacleBehaviour")
          self.subsystemBehaviour_avoidObstacleBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_collectBallBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_collectBallBehaviour")
          self.subsystemBehaviour_collectBallBehaviour()
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

  ##### Update data for input buffer: obstacleDetectedAuto #####
  def update_obstacleDetectedAuto(self, data):
    self.log("update_obstacleDetectedAuto")
    self.obstacleDetectedAuto=data
    self._in_flag_obstacleDetectedAuto=True
    pass

  ##### Update data for input buffer: ballCollected #####
  def update_ballCollected(self, data):
    self.log("update_ballCollected")
    self.ballCollected=data
    self._in_flag_ballCollected=True
    pass

  ##### Update data for input buffer: bestBallDetectedIntel #####
  def update_bestBallDetectedIntel(self, data):
    self.log("update_bestBallDetectedIntel")
    self.bestBallDetectedIntel=data
    self._in_flag_bestBallDetectedIntel=True
    pass

  ##### Update data for input buffer: bestBallDetectedRpi #####
  def update_bestBallDetectedRpi(self, data):
    self.log("update_bestBallDetectedRpi")
    self.bestBallDetectedRpi=data
    self._in_flag_bestBallDetectedRpi=True
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
    self._sender_taskStatus=rospy.Publisher("taskStatusChannelAuto", String, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredRobotSpeedAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredRobotSpeedAuto=rospy.Publisher("desiredRobotSpeedChannelAuto", Int64, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredRobotCommandAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredRobotCommandAuto=rospy.Publisher("desiredRobotCommandChannelAuto", String, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredVaccumSpeedAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredVaccumSpeedAuto=rospy.Publisher("desiredVaccumSpeedChannelAuto", Int64, queue_size=CHANNEL_SIZE)
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
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/obstacleDetected', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/isBallCollected', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_taskCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_obstacleDetectedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_robotStatus', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_ballCollected', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_bestBallDetectedIntel', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_bestBallDetectedRpi', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_taskStatus', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredRobotSpeedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredRobotCommandAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredVaccumSpeedAuto', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=taskCommand sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_taskCommand=rospy.Subscriber("taskCommandChannel", String, self.update_taskCommand)
    # Buffer name=obstacleDetectedAuto sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_obstacleDetectedAuto=rospy.Subscriber("obstacleDetectedChannelAuto", ObstacleDetected, self.update_obstacleDetectedAuto)
    # Buffer name=ballCollected sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_ballCollected=rospy.Subscriber("ballCollectedChannelAuto", Bool, self.update_ballCollected)
    # Buffer name=bestBallDetectedIntel sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_bestBallDetectedIntel=rospy.Subscriber("bestBallDetectedIntelChannel", CameraMessage, self.update_bestBallDetectedIntel)
    # Buffer name=bestBallDetectedRpi sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_bestBallDetectedRpi=rospy.Subscriber("bestBallDetectedRpiChannel", CameraMessage, self.update_bestBallDetectedRpi)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", ObstacleDetected,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Bool,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", CameraMessage,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", CameraMessage,  timeout=TOPIC_TIMEOUT)
    pass

  ##### Publish on topics diagnostic data concerning the subsystem state #####
  def sendDataForDiagnostics(self):
    self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemBehaviour)
    self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
    self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
    self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
    self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
    ###### internal state #####
    if(16 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self.obstacleDetected)
      self._vectorOfSenderDiagnostics[6].publish(self.isBallCollected)
      self._vectorOfSenderDiagnostics[7].publish(self._in_flag_taskCommand)
      self._vectorOfSenderDiagnostics[8].publish(self._in_flag_obstacleDetectedAuto)
      self._vectorOfSenderDiagnostics[9].publish(self._in_flag_robotStatus)
      self._vectorOfSenderDiagnostics[10].publish(self._in_flag_ballCollected)
      self._vectorOfSenderDiagnostics[11].publish(self._in_flag_bestBallDetectedIntel)
      self._vectorOfSenderDiagnostics[12].publish(self._in_flag_bestBallDetectedRpi)
      self._vectorOfSenderDiagnostics[13].publish(self._out_flag_taskStatus)
      self._vectorOfSenderDiagnostics[14].publish(self._out_flag_desiredRobotSpeedAuto)
      self._vectorOfSenderDiagnostics[15].publish(self._out_flag_desiredRobotCommandAuto)
      self._vectorOfSenderDiagnostics[16].publish(self._out_flag_desiredVaccumSpeedAuto)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
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
    try:
      taskAgentObstacleMaxDistance =rospy.get_param("/taskAgentObstacleMaxDistance")
    except rospy.ROSException:
      taskAgentObstacleMaxDistance=15
    except Exception as e:
      taskAgentObstacleMaxDistance=15
    print("[CS - autonomous] -- initBehaviour")
    self.taskCommand=String("empty")
    self.taskStatus=String("empty")    
    self.obstacleDetectedAuto=ObstacleDetected()
    self.ballCollected=Bool(False)
    print("AAAAAAAAA")
    #self.ballInfoRPi = CameraMessage()
    print("BBBBBBBBBBB")
    #self.ballInfoRPi.ballVisible=Bool(False)
    #self.ballInfoRPi.ballPosition=Point(0,0,0)
    print("CCCCCCCCC")
    # self.isBallVisible=Bool(False)
    self.obstacleDetected=Int64(-1)
    print("CCCCCCCCC 1")
    self.desiredRobotSpeedAuto=Int64(0)
    self.desiredRobotCommandAuto=String("empty")
    self.desiredVaccumSpeedAuto=Int64(VACUUM_TURN_OFF)
    print("CCCCCCCCC 2")
    self.robotStatus=String("empty")
    self.isBallCollected=Bool(False)
    print("CCCCCCCCC 3")
    self.bestBallDetectedIntel=CameraMessage()
    self.bestBallDetectedIntel.ballVisible=Bool(False)
    self.bestBallDetectedRpi=CameraMessage()
    self.bestBallDetectedRpi.ballVisible=Bool(False)
    print("CCCCCCCCC 4")
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
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
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
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
  def terminalCondition_idleBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
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
    print("[CS - autonomous] -- idleBehaviour")
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
    # check if output buffer taskStatus has new data - i.e. is ready to send new data
    if( self._out_flag_taskStatus ):
      # send data from output buffer taskStatus
      # Buffer taskStatus - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_taskStatus.publish(self.taskStatus) # sending data from output buffer taskStatus #
      # indicate that data was sent and now the output buffer taskStatus is empty
      self._out_flag_taskStatus=False
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
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
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
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
  def terminalCondition_terminateBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour terminateBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_terminateBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
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
    self._out_flag_desiredRobotCommandAuto=True
    self._out_flag_desiredRobotSpeedAuto=True
    self._out_flag_desiredVaccumSpeedAuto=True
    print("[CS - autonomous] -- terminateBehaviour")
    self.taskStatus.data="task terminated"
    self.desiredRobotCommandAuto.data="terminate"
    self.desiredRobotSpeedAuto.data=0    
    self.desiredVaccumSpeedAuto.data=VACUUM_TURN_OFF
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
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
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
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
  def terminalCondition_terminatedBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour terminatedBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_terminatedBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
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
    print("[CS - autonomous] -- terminatedBehaviour")
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
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
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
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

  ##### Behaviour findBallBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_findBallBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour findBallBehaviour] -- Checking Terminal Condition")
    return  ( self.bestBallDetectedIntel.ballVisible.data or self.bestBallDetectedRpi.ballVisible.data ) or self.obstacleDetected.data>=0 or self.taskCommand.data=="terminate" 
    pass

  ##### Error condition #####
  def errorCondition_findBallBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour findBallBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_findBallBehaviour(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - findBallBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_findBallBehaviour_fun1()
    # Partial transition function call: fun2
    self.transitionFunction_findBallBehaviour_fun2()
    # Partial transition function call: fun3
    self.transitionFunction_findBallBehaviour_fun3()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_findBallBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_findBallBehaviour_fun1(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_findBallBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_findBallBehaviour_fun1_0(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandAuto=True
    self._out_flag_desiredRobotSpeedAuto=True
    self._out_flag_desiredVaccumSpeedAuto=True
    try:
      taskAgentObstacleMaxDistance =rospy.get_param("/taskAgentObstacleMaxDistance")
    except rospy.ROSException:
      return
    except Exception as e:
      taskAgentObstacleMaxDistance=15
      return
    print("[CS - autonomous] -- findBallBehaviour")  
    self.desiredVaccumSpeedAuto.data=VACUUM_TURN_OFF
    if(self.taskStatus.data=="task status: moving front to find a ball"):
      self._out_flag_taskStatus=False
    if(( self.bestBallDetectedIntel.ballVisible.data or self.bestBallDetectedRpi.ballVisible.data )):
      #self.isBallVisible.data=True
      self.taskStatus.data="task status: ball found"
    else:
      #self.isBallVisible.data=False
      self.desiredRobotCommandAuto.data="rotate right"
      self.desiredRobotSpeedAuto.data=v_rot_min
      self.taskStatus.data="task status: rotating right to find a ball"
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_findBallBehaviour_fun2(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_obstacleDetectedAuto:
      self.transitionFunction_findBallBehaviour_fun2_0()
    elif  not (self._in_flag_obstacleDetectedAuto):
      self.transitionFunction_findBallBehaviour_fun2_1()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_obstacleDetectedAuto #####
  def transitionFunction_findBallBehaviour_fun2_0(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - autonomous] -- findBallBehaviour")
    # internal memory
    self.obstacleDetected.data=checkIfObstacleDetected(self.obstacleDetectedAuto)
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not (self._in_flag_obstacleDetectedAuto) #####
  def transitionFunction_findBallBehaviour_fun2_1(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_findBallBehaviour_fun3(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun3")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_ballCollected:
      self.transitionFunction_findBallBehaviour_fun3_0()
    elif  not (self._in_flag_ballCollected):
      self.transitionFunction_findBallBehaviour_fun3_1()
    pass

  ##### Partial transition function: fun3_0 based on input buffers self._in_flag_ballCollected #####
  def transitionFunction_findBallBehaviour_fun3_0(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun3_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - autonomous] -- findBallBehaviour")
    # internal memory
    self.isBallCollected.data=self.ballCollected.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun3_1 based on input buffers  not (self._in_flag_ballCollected) #####
  def transitionFunction_findBallBehaviour_fun3_1(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function fun3_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_findBallBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_findBallBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_findBallBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour findBallBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - findBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_obstacleDetectedAuto=False
    self._in_flag_ballCollected=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_findBallBehaviour(self):
    self.log("[Behaviour findBallBehaviour] -- Sending Data")
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_findBallBehaviour(self):
    self.log("[Behaviour findBallBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour findBallBehaviour #####
  def executeBehaviour_findBallBehaviour(self):
    self.log("[Behaviour findBallBehaviour] -- Executing findBallBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour findBallBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_findBallBehaviour()
      # Sends data! #
      self.sendData_findBallBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_findBallBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_findBallBehaviour() or self.errorCondition_findBallBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour avoidObstacleBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_avoidObstacleBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour avoidObstacleBehaviour] -- Checking Terminal Condition")
    return  not self.obstacleDetected.data>=0 or self.taskCommand.data=="terminate" 
    pass

  ##### Error condition #####
  def errorCondition_avoidObstacleBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour avoidObstacleBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_avoidObstacleBehaviour(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_avoidObstacleBehaviour_fun1()
    # Partial transition function call: fun2
    self.transitionFunction_avoidObstacleBehaviour_fun2()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_avoidObstacleBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_avoidObstacleBehaviour_fun1(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_avoidObstacleBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_avoidObstacleBehaviour_fun1_0(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    try:
      taskAgentObstacleMaxDistance =rospy.get_param("/taskAgentObstacleMaxDistance")
    except rospy.ROSException:
      print("BBBBBBBBBBBBBBBBBBB")
      print("BBBBBBBBBBBBBBBBBBB")
      print("BBBBBBBBBBBBBBBBBBB")
      return
    except Exception as e:
      print("AAAAAAAAAAAAAAAAAAAAAAAA")
      print("BBBBBBBBBBBBBBBBBBBBBBBB")
      print("CCCCCCCCCCCCCCCCCCCCCCCCC")
      taskAgentObstacleMaxDistance=15
      return
      print("[CS - autonomous] -- avoidObstacleBehaviour")
      self.taskStatus.data="task status: avoid obstacle"
    if(self.taskStatus.data=="task status: avoid obstacle"):
      self._out_flag_taskStatus=False
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_avoidObstacleBehaviour_fun2(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_obstacleDetectedAuto:
      self.transitionFunction_avoidObstacleBehaviour_fun2_0()
    elif  not (self._in_flag_obstacleDetectedAuto):
      self.transitionFunction_avoidObstacleBehaviour_fun2_1()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_obstacleDetectedAuto #####
  def transitionFunction_avoidObstacleBehaviour_fun2_0(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandAuto=True
    self._out_flag_desiredRobotSpeedAuto=True
    self._out_flag_desiredVaccumSpeedAuto=True
    print("[CS - autonomous] -- avoidObstacleBehaviour")
    self.obstacleDetected.data=checkIfObstacleDetected(self.obstacleDetectedAuto)
    self.desiredVaccumSpeedAuto.data=VACUUM_TURN_OFF
    if(self.obstacleDetected.data>=0):
      # stop robot - just for now ############################### CHANGE LATER
      self.desiredRobotCommandAuto.data="stop"
      self.desiredRobotSpeedAuto.data=0
      self.taskStatus.data="empty"
    else:
      # obstacle was avoided
      self.taskStatus.data="task status: obstacle avoided"
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not (self._in_flag_obstacleDetectedAuto) #####
  def transitionFunction_avoidObstacleBehaviour_fun2_1(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_avoidObstacleBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_avoidObstacleBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_avoidObstacleBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour avoidObstacleBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - avoidObstacleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_obstacleDetectedAuto=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_avoidObstacleBehaviour(self):
    self.log("[Behaviour avoidObstacleBehaviour] -- Sending Data")
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_avoidObstacleBehaviour(self):
    self.log("[Behaviour avoidObstacleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour avoidObstacleBehaviour #####
  def executeBehaviour_avoidObstacleBehaviour(self):
    self.log("[Behaviour avoidObstacleBehaviour] -- Executing avoidObstacleBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour avoidObstacleBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_avoidObstacleBehaviour()
      # Sends data! #
      self.sendData_avoidObstacleBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_avoidObstacleBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_avoidObstacleBehaviour() or self.errorCondition_avoidObstacleBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour collectBallBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_collectBallBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour collectBallBehaviour] -- Checking Terminal Condition")
    return  self.obstacleDetected.data>=0 or self.taskCommand.data=="terminate" or ( not self.bestBallDetectedIntel.ballVisible.data and not self.bestBallDetectedRpi.ballVisible.data ) 
    pass

  ##### Error condition #####
  def errorCondition_collectBallBehaviour(self): # String taskCommand, ObstacleDetected obstacleDetectedAuto, String robotStatus, Bool ballCollected, CameraMessage bestBallDetectedIntel, CameraMessage bestBallDetectedRpi, Int64 obstacleDetected, Bool isBallCollected, std_msgs::Bool _in_flag_taskCommand, std_msgs::Bool _in_flag_obstacleDetectedAuto, std_msgs::Bool _in_flag_robotStatus, std_msgs::Bool _in_flag_ballCollected, std_msgs::Bool _in_flag_bestBallDetectedIntel, std_msgs::Bool _in_flag_bestBallDetectedRpi, std_msgs::Bool _out_flag_taskStatus, std_msgs::Bool _out_flag_desiredRobotSpeedAuto, std_msgs::Bool _out_flag_desiredRobotCommandAuto, std_msgs::Bool _out_flag_desiredVaccumSpeedAuto #
    self.log("[Behaviour collectBallBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_collectBallBehaviour(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - collectBallBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_collectBallBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_collectBallBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_collectBallBehaviour_fun1(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - collectBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_ballCollected:
      self.transitionFunction_collectBallBehaviour_fun1_0()
    elif  not (self._in_flag_ballCollected):
      self.transitionFunction_collectBallBehaviour_fun1_1()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_ballCollected #####
  def transitionFunction_collectBallBehaviour_fun1_0(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - collectBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_taskStatus=True
    self._out_flag_desiredRobotCommandAuto=True
    self._out_flag_desiredRobotSpeedAuto=True
    self._out_flag_desiredVaccumSpeedAuto=True
    print("[CS - autonomous] -- collectBallBehaviour")
    try:
      IMAGE_X_SIZE = rospy.get_param("/imageXSize")
      IMAGE_Y_SIZE = rospy.get_param("/imageYSize")
      minRpiInletX =rospy.get_param("/minRpiInletX")
      v_rot_max = rospy.get_param("/vRotMax")
      v_rot_min =rospy.get_param("/vRotMin")
      v_front_max =rospy.get_param("/vFrontMax")
      v_front_min =rospy.get_param("/vFrontMin")
      VACUUM_TURN_ON =rospy.get_param("/vacuumTurnOn")
      print("AAAAAAAAAAAAAAAAAAAAAAAA")
    except rospy.ROSException:
      print("BBBBBBBBBBBBBBBBBBB")
      print("BBBBBBBBBBBBBBBBBBB")
      print("BBBBBBBBBBBBBBBBBBB")
      return
    except Exception as e:
      print("AAAAAAAAAAAAAAAAAAAAAAAA")
      print("BBBBBBBBBBBBBBBBBBBBBBBB")
      print("AAAAAAAAAAAAAAAAAAAAAAAA")
      print("error=",e)
      IMAGE_X_SIZE = 640
      IMAGE_Y_SIZE = 480
      minRpiInletX =100
      v_rot_max = 10
      v_rot_min =5
      v_front_max =10
      v_front_min =12
      VACUUM_TURN_ON =150
      return
    self.taskStatus.data=="task status: collect ball"
    # check if obstacle is detected
    self.obstacleDetected.data=checkIfObstacleDetected(self.obstacleDetectedAuto)
    # get ball position from RRi camera:
    if(( not self.bestBallDetectedIntel.ballVisible.data and not self.bestBallDetectedRpi.ballVisible.data )):
      # ball is not visible - stop behaviour
      return
    ################################
    # TODO ################
    # isRpiModeActivated indicates the control mode based on camera where the ball was detected. If isRpiModeActivated==True then it means that the ball is visible in RPi camera (and it can also be visible in intel camera), if False that the ball is visible only in intel camera
    isRpiModeActivated=False
    # check if ball is detected in rpi camera
    if(self.bestBallDetectedRpi.ballVisible.data):
      # if yes control robot based on position of best ball detected by rpi camera 
      isRpiModeActivated=True
    # else (ball is not detected by rpi camera)
    # check if ball is detected in intel camera
    elif(self.bestBallDetectedIntel.ballVisible.data):
      # if yes control robot based on position of best ball detected by intel camera
      isRpiModeActivated=False
    ################################
    # ball is detected (within RPi camera)
    # check in which direction the robot should rotate
    velocity=0
    if(isRpiModeActivated):
      midXPointRpi=IMAGE_X_SIZE/2 - 40
      deltaX = midXPointRpi - self.bestBallDetectedRpi.ballPosition.x ## todo ....
      deltaY = IMAGE_Y_SIZE - self.bestBallDetectedRpi.ballPosition.y ## todo ....
    else:
      midXPointRpi=IMAGE_X_SIZE/2
      deltaX = midXPointRpi - self.bestBallDetectedIntel.ballPosition.x ## todo ....
      deltaY = IMAGE_Y_SIZE - self.bestBallDetectedIntel.ballPosition.y ## todo ....
      velocity=v_front_max
    command="empty"
    if(deltaX>0):
      # rotate left
      command="rotate left"
    else:
      # rotate right
      command="rotate right"
    if( abs(deltaX) < minRpiInletX and isRpiModeActivated):
      # move front
      command="move front"
      # set low velocity
      # ball detected in robot vicinity (rpi camera)
      #velocity=v_front_max * (abs(deltaY)/IMAGE_Y_SIZE) + v_front_min
      velocity=v_front_min
      # here the robot vacuum cleaner should be activated
      self.desiredVaccumSpeedAuto.data=VACUUM_TURN_ON
    elif( abs(deltaX) < 2*minRpiInletX and not isRpiModeActivated):
      # move front
      command="move front"
      # ball detected far from the robot vicinity (intel camera)
      # set normal velocity
      #velocity=v_front_max
      velocity=v_front_min
      # here the robot vacuum cleaner should be deactivated
      self.desiredVaccumSpeedAuto.data=VACUUM_TURN_OFF
    else:
      # robot rotates with speed based on ball position
      # calculate velocity
      print("velocity=",velocity)
      print("v_rot_max=",v_rot_max)
      print("v_rot_max * (abs(deltaX)/(IMAGE_X_SIZE/2))=",v_rot_max * (abs(deltaX)/(IMAGE_X_SIZE/2)))
      print("v_rot_min=",v_rot_min)
      #velocity=v_rot_max * (abs(deltaX)/(IMAGE_X_SIZE/2)) + v_rot_min
      velocity=v_rot_min
      # here the robot vacuum cleaner should be deactivated
      self.desiredVaccumSpeedAuto.data=VACUUM_TURN_OFF
    # set velocity and command
    print("AB BBBBBBBBBB velocity=",velocity)
    print("AB BBBBBBBBBB command=",command)
    self.desiredRobotSpeedAuto.data=velocity
    self.desiredRobotCommandAuto.data=command  
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not (self._in_flag_ballCollected) #####
  def transitionFunction_collectBallBehaviour_fun1_1(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - collectBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_collectBallBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - collectBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_collectBallBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_collectBallBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour collectBallBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - collectBallBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_ballCollected=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_collectBallBehaviour(self):
    self.log("[Behaviour collectBallBehaviour] -- Sending Data")
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
    # check if output buffer desiredRobotSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotSpeedAuto ):
      # send data from output buffer desiredRobotSpeedAuto
      # Buffer desiredRobotSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotSpeedAuto.publish(self.desiredRobotSpeedAuto) # sending data from output buffer desiredRobotSpeedAuto #
      # indicate that data was sent and now the output buffer desiredRobotSpeedAuto is empty
      self._out_flag_desiredRobotSpeedAuto=False
    # check if output buffer desiredRobotCommandAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredRobotCommandAuto ):
      # send data from output buffer desiredRobotCommandAuto
      # Buffer desiredRobotCommandAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredRobotCommandAuto.publish(self.desiredRobotCommandAuto) # sending data from output buffer desiredRobotCommandAuto #
      # indicate that data was sent and now the output buffer desiredRobotCommandAuto is empty
      self._out_flag_desiredRobotCommandAuto=False
    # check if output buffer desiredVaccumSpeedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVaccumSpeedAuto ):
      # send data from output buffer desiredVaccumSpeedAuto
      # Buffer desiredVaccumSpeedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVaccumSpeedAuto.publish(self.desiredVaccumSpeedAuto) # sending data from output buffer desiredVaccumSpeedAuto #
      # indicate that data was sent and now the output buffer desiredVaccumSpeedAuto is empty
      self._out_flag_desiredVaccumSpeedAuto=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_collectBallBehaviour(self):
    self.log("[Behaviour collectBallBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer taskCommand - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer taskCommand
    # Buffer obstacleDetectedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer obstacleDetectedAuto
    # Buffer ballCollected - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer ballCollected
    # Buffer bestBallDetectedIntel - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedIntel
    # Buffer bestBallDetectedRpi - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer bestBallDetectedRpi
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour collectBallBehaviour #####
  def executeBehaviour_collectBallBehaviour(self):
    self.log("[Behaviour collectBallBehaviour] -- Executing collectBallBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour collectBallBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_collectBallBehaviour()
      # Sends data! #
      self.sendData_collectBallBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_collectBallBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_collectBallBehaviour() or self.errorCondition_collectBallBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass


  ##### Definition of functions responsible for switching subsystem cs between states : Behaviour_initBehaviour Behaviour_idleBehaviour Behaviour_terminateBehaviour Behaviour_terminatedBehaviour Behaviour_findBallBehaviour Behaviour_avoidObstacleBehaviour Behaviour_collectBallBehaviour  #####
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
    # Checking initial condition for behaviour idleBehaviour: switching to behaviour findBallBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_findBallBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_findBallBehaviour"

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


  # Behaviour findBallBehaviour: #
  def subsystemBehaviour_findBallBehaviour(self):
    self.log("subsystemBehaviour_findBallBehaviour")
    # Executing behaviour findBallBehaviour #
    self.executeBehaviour_findBallBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour findBallBehaviour: switching to behaviour collectBallBehaviour #
    if self.initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_collectBallBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_collectBallBehaviour"

    # Checking initial condition for behaviour findBallBehaviour: switching to behaviour avoidObstacleBehaviour #
    if self.initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_avoidObstacleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_avoidObstacleBehaviour"

    # Checking initial condition for behaviour findBallBehaviour: switching to behaviour terminateBehaviour #
    if self.initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_terminateBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminateBehaviour"

    pass


  # Behaviour avoidObstacleBehaviour: #
  def subsystemBehaviour_avoidObstacleBehaviour(self):
    self.log("subsystemBehaviour_avoidObstacleBehaviour")
    # Executing behaviour avoidObstacleBehaviour #
    self.executeBehaviour_avoidObstacleBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour avoidObstacleBehaviour: switching to behaviour findBallBehaviour #
    if self.initialCondition_From_Behaviour_avoidObstacleBehaviour_To_Behaviour_findBallBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_findBallBehaviour"

    # Checking initial condition for behaviour avoidObstacleBehaviour: switching to behaviour terminateBehaviour #
    if self.initialCondition_From_Behaviour_avoidObstacleBehaviour_To_Behaviour_terminateBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminateBehaviour"

    pass


  # Behaviour collectBallBehaviour: #
  def subsystemBehaviour_collectBallBehaviour(self):
    self.log("subsystemBehaviour_collectBallBehaviour")
    # Executing behaviour collectBallBehaviour #
    self.executeBehaviour_collectBallBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour collectBallBehaviour: switching to behaviour terminateBehaviour #
    if self.initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_terminateBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_terminateBehaviour"

    # Checking initial condition for behaviour collectBallBehaviour: switching to behaviour avoidObstacleBehaviour #
    if self.initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_avoidObstacleBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_avoidObstacleBehaviour"

    # Checking initial condition for behaviour collectBallBehaviour: switching to behaviour findBallBehaviour #
    if self.initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_findBallBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_findBallBehaviour"

    pass


  ##### Initial condition for behaviour initBehaviour: switching to behaviour idleBehaviour #####
  def initialCondition_From_Behaviour_initBehaviour_To_Behaviour_idleBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour findBallBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_findBallBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour terminateBehaviour: switching to behaviour terminatedBehaviour #####
  def initialCondition_From_Behaviour_terminateBehaviour_To_Behaviour_terminatedBehaviour(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour findBallBehaviour: switching to behaviour collectBallBehaviour #####
  def initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_collectBallBehaviour(self):
    # Initial condition specified by user #
    return  ( self.bestBallDetectedIntel.ballVisible.data or self.bestBallDetectedRpi.ballVisible.data ) and not (self.obstacleDetected.data>=0) 


  ##### Initial condition for behaviour findBallBehaviour: switching to behaviour avoidObstacleBehaviour #####
  def initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_avoidObstacleBehaviour(self):
    # Initial condition specified by user #
    return  self.obstacleDetected.data>=0 


  ##### Initial condition for behaviour findBallBehaviour: switching to behaviour terminateBehaviour #####
  def initialCondition_From_Behaviour_findBallBehaviour_To_Behaviour_terminateBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="terminate" 


  ##### Initial condition for behaviour avoidObstacleBehaviour: switching to behaviour findBallBehaviour #####
  def initialCondition_From_Behaviour_avoidObstacleBehaviour_To_Behaviour_findBallBehaviour(self):
    # Initial condition specified by user #
    return  not self.taskCommand.data=="terminate" 


  ##### Initial condition for behaviour avoidObstacleBehaviour: switching to behaviour terminateBehaviour #####
  def initialCondition_From_Behaviour_avoidObstacleBehaviour_To_Behaviour_terminateBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="terminate" 


  ##### Initial condition for behaviour collectBallBehaviour: switching to behaviour terminateBehaviour #####
  def initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_terminateBehaviour(self):
    # Initial condition specified by user #
    return  self.taskCommand.data=="terminate" 


  ##### Initial condition for behaviour collectBallBehaviour: switching to behaviour avoidObstacleBehaviour #####
  def initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_avoidObstacleBehaviour(self):
    # Initial condition specified by user #
    return  self.obstacleDetected.data>=0 


  ##### Initial condition for behaviour collectBallBehaviour: switching to behaviour findBallBehaviour #####
  def initialCondition_From_Behaviour_collectBallBehaviour_To_Behaviour_findBallBehaviour(self):
    # Initial condition specified by user #
    return  ( self.ballCollected.data or ( not self.bestBallDetectedIntel.ballVisible.data and not self.bestBallDetectedRpi.ballVisible.data ) ) and not (self.obstacleDetected.data>=0) 



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

