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
from auxiliary_agent_ballcollector import *
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
    self._in_flag_desiredRobotCommandTele=False
    self._in_flag_desiredRobotCommandAuto=False
    self._in_flag_cameraInfo=False
    self._in_flag_sensorInfo=False
    self._in_flag_detectedBalls=False
    self._in_flag_rpiCamera=False
    self._in_flag_desiredRobotSpeedAuto=False
    self._in_flag_desiredVaccumSpeedAuto=False
    # initialize all output flags
    self._out_flag_obstacleDetectedAuto=False
    self._out_flag_obstacleDetectedTele=False
    self._out_flag_ballInfoAuto=False
    self._out_flag_ballInfoTele=False
    self._out_flag_ballCollectedTele=False
    self._out_flag_ballCollectedAuto=False
    self._out_flag_desiredVacuumCommand=False
    self._out_flag_desiredMoveCommand=False
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

  ##### Update data for input buffer: desiredRobotCommandTele #####
  def update_desiredRobotCommandTele(self, data):
    self.log("update_desiredRobotCommandTele")
    self.desiredRobotCommandTele=data
    self._in_flag_desiredRobotCommandTele=True
    pass

  ##### Update data for input buffer: desiredRobotCommandAuto #####
  def update_desiredRobotCommandAuto(self, data):
    self.log("update_desiredRobotCommandAuto")
    self.desiredRobotCommandAuto=data
    self._in_flag_desiredRobotCommandAuto=True
    pass

  ##### Update data for input buffer: cameraInfo #####
  def update_cameraInfo(self, data):
    self.log("update_cameraInfo")
    self.cameraInfo=data
    self._in_flag_cameraInfo=True
    pass

  ##### Update data for input buffer: sensorInfo #####
  def update_sensorInfo(self, data):
    self.log("update_sensorInfo")
    self.sensorInfo=data
    self._in_flag_sensorInfo=True
    pass

  ##### Update data for input buffer: detectedBalls #####
  def update_detectedBalls(self, data):
    self.log("update_detectedBalls")
    self.detectedBalls=data
    self._in_flag_detectedBalls=True
    pass

  ##### Update data for input buffer: rpiCamera #####
  def update_rpiCamera(self, data):
    self.log("update_rpiCamera")
    self.rpiCamera=data
    self._in_flag_rpiCamera=True
    pass

  ##### Update data for input buffer: desiredRobotSpeedAuto #####
  def update_desiredRobotSpeedAuto(self, data):
    self.log("update_desiredRobotSpeedAuto")
    self.desiredRobotSpeedAuto=data
    self._in_flag_desiredRobotSpeedAuto=True
    pass

  ##### Update data for input buffer: desiredVaccumSpeedAuto #####
  def update_desiredVaccumSpeedAuto(self, data):
    self.log("update_desiredVaccumSpeedAuto")
    self.desiredVaccumSpeedAuto=data
    self._in_flag_desiredVaccumSpeedAuto=True
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
    # Buffer name=obstacleDetectedAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_obstacleDetectedAuto=rospy.Publisher("obstacleDetectedChannelAuto", ObstacleDetected, queue_size=CHANNEL_SIZE)
    # Buffer name=obstacleDetectedTele - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_obstacleDetectedTele=rospy.Publisher("obstacleDetectedChannelTele", ObstacleDetected, queue_size=CHANNEL_SIZE)
    # Buffer name=ballInfoAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_ballInfoAuto=rospy.Publisher("ballInfoRpiChannelAuto", CameraMessage, queue_size=CHANNEL_SIZE)
    # Buffer name=ballInfoTele - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_ballInfoTele=rospy.Publisher("ballInfoChannelTele", CameraMessage, queue_size=CHANNEL_SIZE)
    # Buffer name=ballCollectedTele - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_ballCollectedTele=rospy.Publisher("ballCollectedChannelTele", Bool, queue_size=CHANNEL_SIZE)
    # Buffer name=ballCollectedAuto - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_ballCollectedAuto=rospy.Publisher("ballCollectedChannelAuto", Bool, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredVacuumCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredVacuumCommand=rospy.Publisher("desiredVacuumCommandChannel", Int64, queue_size=CHANNEL_SIZE)
    # Buffer name=desiredMoveCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_desiredMoveCommand=rospy.Publisher("desiredMoveCommandChannel", MotorMessage, queue_size=CHANNEL_SIZE)
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
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/currentRobotSpeed', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_desiredRobotCommandTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_desiredRobotCommandAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_cameraInfo', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_sensorInfo', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_detectedBalls', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_rpiCamera', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_desiredRobotSpeedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_desiredVaccumSpeedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_obstacleDetectedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_obstacleDetectedTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_ballInfoAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_ballInfoTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_ballCollectedTele', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_ballCollectedAuto', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredVacuumCommand', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_desiredMoveCommand', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=desiredRobotCommandTele sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_desiredRobotCommandTele=rospy.Subscriber("desiredRobotCommandChannelTele", String, self.update_desiredRobotCommandTele)
    # Buffer name=desiredRobotCommandAuto sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_desiredRobotCommandAuto=rospy.Subscriber("desiredRobotCommandChannelAuto", String, self.update_desiredRobotCommandAuto)
    # Buffer name=cameraInfo sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_cameraInfo=rospy.Subscriber("cameraInfoChannel", CameraMessage, self.update_cameraInfo)
    # Buffer name=sensorInfo sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_sensorInfo=rospy.Subscriber("sensorInfoChannel", SensorMessage, self.update_sensorInfo)
    # Buffer name=detectedBalls sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_detectedBalls=rospy.Subscriber("detectedBallsChannel", Image, self.update_detectedBalls)
    # Buffer name=rpiCamera sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_rpiCamera=rospy.Subscriber("rpiCameraChannel", Image, self.update_rpiCamera)
    # Buffer name=desiredRobotSpeedAuto sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_desiredRobotSpeedAuto=rospy.Subscriber("desiredRobotSpeedChannelAuto", Int64, self.update_desiredRobotSpeedAuto)
    # Buffer name=desiredVaccumSpeedAuto sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_desiredVaccumSpeedAuto=rospy.Subscriber("desiredVaccumSpeedChannelAuto", Int64, self.update_desiredVaccumSpeedAuto)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", String,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", CameraMessage,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", SensorMessage,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Image,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Image,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Int64,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Int64,  timeout=TOPIC_TIMEOUT)
    pass

  ##### Publish on topics diagnostic data concerning the subsystem state #####
  def sendDataForDiagnostics(self):
    self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemBehaviour)
    self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
    self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
    self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
    self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
    ###### internal state #####
    if(21 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self.currentRobotSpeed)
      self._vectorOfSenderDiagnostics[6].publish(self._in_flag_desiredRobotCommandTele)
      self._vectorOfSenderDiagnostics[7].publish(self._in_flag_desiredRobotCommandAuto)
      self._vectorOfSenderDiagnostics[8].publish(self._in_flag_cameraInfo)
      self._vectorOfSenderDiagnostics[9].publish(self._in_flag_sensorInfo)
      self._vectorOfSenderDiagnostics[10].publish(self._in_flag_detectedBalls)
      self._vectorOfSenderDiagnostics[11].publish(self._in_flag_rpiCamera)
      self._vectorOfSenderDiagnostics[12].publish(self._in_flag_desiredRobotSpeedAuto)
      self._vectorOfSenderDiagnostics[13].publish(self._in_flag_desiredVaccumSpeedAuto)
      self._vectorOfSenderDiagnostics[14].publish(self._out_flag_obstacleDetectedAuto)
      self._vectorOfSenderDiagnostics[15].publish(self._out_flag_obstacleDetectedTele)
      self._vectorOfSenderDiagnostics[16].publish(self._out_flag_ballInfoAuto)
      self._vectorOfSenderDiagnostics[17].publish(self._out_flag_ballInfoTele)
      self._vectorOfSenderDiagnostics[18].publish(self._out_flag_ballCollectedTele)
      self._vectorOfSenderDiagnostics[19].publish(self._out_flag_ballCollectedAuto)
      self._vectorOfSenderDiagnostics[20].publish(self._out_flag_desiredVacuumCommand)
      self._vectorOfSenderDiagnostics[21].publish(self._out_flag_desiredMoveCommand)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # String desiredRobotCommandTele, String desiredRobotCommandAuto, CameraMessage cameraInfo, SensorMessage sensorInfo, Image detectedBalls, Image rpiCamera, Int64 desiredRobotSpeedAuto, Int64 desiredVaccumSpeedAuto, Int64 currentRobotSpeed, std_msgs::Bool _in_flag_desiredRobotCommandTele, std_msgs::Bool _in_flag_desiredRobotCommandAuto, std_msgs::Bool _in_flag_cameraInfo, std_msgs::Bool _in_flag_sensorInfo, std_msgs::Bool _in_flag_detectedBalls, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _in_flag_desiredRobotSpeedAuto, std_msgs::Bool _in_flag_desiredVaccumSpeedAuto, std_msgs::Bool _out_flag_obstacleDetectedAuto, std_msgs::Bool _out_flag_obstacleDetectedTele, std_msgs::Bool _out_flag_ballInfoAuto, std_msgs::Bool _out_flag_ballInfoTele, std_msgs::Bool _out_flag_ballCollectedTele, std_msgs::Bool _out_flag_ballCollectedAuto, std_msgs::Bool _out_flag_desiredVacuumCommand, std_msgs::Bool _out_flag_desiredMoveCommand #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # String desiredRobotCommandTele, String desiredRobotCommandAuto, CameraMessage cameraInfo, SensorMessage sensorInfo, Image detectedBalls, Image rpiCamera, Int64 desiredRobotSpeedAuto, Int64 desiredVaccumSpeedAuto, Int64 currentRobotSpeed, std_msgs::Bool _in_flag_desiredRobotCommandTele, std_msgs::Bool _in_flag_desiredRobotCommandAuto, std_msgs::Bool _in_flag_cameraInfo, std_msgs::Bool _in_flag_sensorInfo, std_msgs::Bool _in_flag_detectedBalls, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _in_flag_desiredRobotSpeedAuto, std_msgs::Bool _in_flag_desiredVaccumSpeedAuto, std_msgs::Bool _out_flag_obstacleDetectedAuto, std_msgs::Bool _out_flag_obstacleDetectedTele, std_msgs::Bool _out_flag_ballInfoAuto, std_msgs::Bool _out_flag_ballInfoTele, std_msgs::Bool _out_flag_ballCollectedTele, std_msgs::Bool _out_flag_ballCollectedAuto, std_msgs::Bool _out_flag_desiredVacuumCommand, std_msgs::Bool _out_flag_desiredMoveCommand #
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
    print("[CS - ballCollector] -- initBehaviour")
    self.desiredMoveCommand=MotorMessage()
    print("ALA")
    self.desiredMoveCommand.desiredSpeed=Int64(10)
    print("ALA1")
    self.desiredMoveCommand.direction=Int64(0)
    print("ALA2")
    self.desiredMoveCommand.cmd=String("empty")
    self.desiredRobotCommandTele=String("empty")
    self.desiredRobotCommandAuto=String("empty")
    self.ballCollectedAuto=Bool(False)
    self.ballCollectedTele=Bool(False)
    self.obstacleDetectedTele=ObstacleDetected()
    self.obstacleDetectedTele.sonar_1=Int64(0)
    self.obstacleDetectedTele.sonar_2=Int64(0)
    self.obstacleDetectedTele.sonar_3=Int64(0)
    self.obstacleDetectedTele.sonar_4=Int64(0)
    self.obstacleDetectedAuto=ObstacleDetected()
    self.obstacleDetectedAuto.sonar_1=Int64(0)
    self.obstacleDetectedAuto.sonar_2=Int64(0)
    self.obstacleDetectedAuto.sonar_3=Int64(0)
    self.obstacleDetectedAuto.sonar_4=Int64(0)
    # vacuum
    self.desiredVacuumCommand=Int64(0)
    # output camera info
    self.ballInfoAuto=CameraMessage()
    self.ballInfoTele=CameraMessage()
    # internal memory
    self.currentRobotSpeed=Int64(0)
    # input buffer
    self.desiredRobotSpeedAuto=Int64(0)
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
    # check if output buffer obstacleDetectedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_obstacleDetectedAuto ):
      # send data from output buffer obstacleDetectedAuto
      # Buffer obstacleDetectedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_obstacleDetectedAuto.publish(self.obstacleDetectedAuto) # sending data from output buffer obstacleDetectedAuto #
      # indicate that data was sent and now the output buffer obstacleDetectedAuto is empty
      self._out_flag_obstacleDetectedAuto=False
    # check if output buffer obstacleDetectedTele has new data - i.e. is ready to send new data
    if( self._out_flag_obstacleDetectedTele ):
      # send data from output buffer obstacleDetectedTele
      # Buffer obstacleDetectedTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_obstacleDetectedTele.publish(self.obstacleDetectedTele) # sending data from output buffer obstacleDetectedTele #
      # indicate that data was sent and now the output buffer obstacleDetectedTele is empty
      self._out_flag_obstacleDetectedTele=False
    # check if output buffer ballInfoAuto has new data - i.e. is ready to send new data
    if( self._out_flag_ballInfoAuto ):
      # send data from output buffer ballInfoAuto
      # Buffer ballInfoAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballInfoAuto.publish(self.ballInfoAuto) # sending data from output buffer ballInfoAuto #
      # indicate that data was sent and now the output buffer ballInfoAuto is empty
      self._out_flag_ballInfoAuto=False
    # check if output buffer ballInfoTele has new data - i.e. is ready to send new data
    if( self._out_flag_ballInfoTele ):
      # send data from output buffer ballInfoTele
      # Buffer ballInfoTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballInfoTele.publish(self.ballInfoTele) # sending data from output buffer ballInfoTele #
      # indicate that data was sent and now the output buffer ballInfoTele is empty
      self._out_flag_ballInfoTele=False
    # check if output buffer ballCollectedTele has new data - i.e. is ready to send new data
    if( self._out_flag_ballCollectedTele ):
      # send data from output buffer ballCollectedTele
      # Buffer ballCollectedTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballCollectedTele.publish(self.ballCollectedTele) # sending data from output buffer ballCollectedTele #
      # indicate that data was sent and now the output buffer ballCollectedTele is empty
      self._out_flag_ballCollectedTele=False
    # check if output buffer ballCollectedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_ballCollectedAuto ):
      # send data from output buffer ballCollectedAuto
      # Buffer ballCollectedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballCollectedAuto.publish(self.ballCollectedAuto) # sending data from output buffer ballCollectedAuto #
      # indicate that data was sent and now the output buffer ballCollectedAuto is empty
      self._out_flag_ballCollectedAuto=False
    # check if output buffer desiredVacuumCommand has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVacuumCommand ):
      # send data from output buffer desiredVacuumCommand
      # Buffer desiredVacuumCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVacuumCommand.publish(self.desiredVacuumCommand) # sending data from output buffer desiredVacuumCommand #
      # indicate that data was sent and now the output buffer desiredVacuumCommand is empty
      self._out_flag_desiredVacuumCommand=False
    # check if output buffer desiredMoveCommand has new data - i.e. is ready to send new data
    if( self._out_flag_desiredMoveCommand ):
      # send data from output buffer desiredMoveCommand
      # Buffer desiredMoveCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredMoveCommand.publish(self.desiredMoveCommand) # sending data from output buffer desiredMoveCommand #
      # indicate that data was sent and now the output buffer desiredMoveCommand is empty
      self._out_flag_desiredMoveCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer desiredRobotCommandTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotCommandTele
    # Buffer desiredRobotCommandAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotCommandAuto
    # Buffer cameraInfo - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer cameraInfo
    # Buffer sensorInfo - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer sensorInfo
    # Buffer detectedBalls - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer detectedBalls
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
    # Buffer desiredRobotSpeedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotSpeedAuto
    # Buffer desiredVaccumSpeedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredVaccumSpeedAuto
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
  def terminalCondition_idleBehaviour(self): # String desiredRobotCommandTele, String desiredRobotCommandAuto, CameraMessage cameraInfo, SensorMessage sensorInfo, Image detectedBalls, Image rpiCamera, Int64 desiredRobotSpeedAuto, Int64 desiredVaccumSpeedAuto, Int64 currentRobotSpeed, std_msgs::Bool _in_flag_desiredRobotCommandTele, std_msgs::Bool _in_flag_desiredRobotCommandAuto, std_msgs::Bool _in_flag_cameraInfo, std_msgs::Bool _in_flag_sensorInfo, std_msgs::Bool _in_flag_detectedBalls, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _in_flag_desiredRobotSpeedAuto, std_msgs::Bool _in_flag_desiredVaccumSpeedAuto, std_msgs::Bool _out_flag_obstacleDetectedAuto, std_msgs::Bool _out_flag_obstacleDetectedTele, std_msgs::Bool _out_flag_ballInfoAuto, std_msgs::Bool _out_flag_ballInfoTele, std_msgs::Bool _out_flag_ballCollectedTele, std_msgs::Bool _out_flag_ballCollectedAuto, std_msgs::Bool _out_flag_desiredVacuumCommand, std_msgs::Bool _out_flag_desiredMoveCommand #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # String desiredRobotCommandTele, String desiredRobotCommandAuto, CameraMessage cameraInfo, SensorMessage sensorInfo, Image detectedBalls, Image rpiCamera, Int64 desiredRobotSpeedAuto, Int64 desiredVaccumSpeedAuto, Int64 currentRobotSpeed, std_msgs::Bool _in_flag_desiredRobotCommandTele, std_msgs::Bool _in_flag_desiredRobotCommandAuto, std_msgs::Bool _in_flag_cameraInfo, std_msgs::Bool _in_flag_sensorInfo, std_msgs::Bool _in_flag_detectedBalls, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _in_flag_desiredRobotSpeedAuto, std_msgs::Bool _in_flag_desiredVaccumSpeedAuto, std_msgs::Bool _out_flag_obstacleDetectedAuto, std_msgs::Bool _out_flag_obstacleDetectedTele, std_msgs::Bool _out_flag_ballInfoAuto, std_msgs::Bool _out_flag_ballInfoTele, std_msgs::Bool _out_flag_ballCollectedTele, std_msgs::Bool _out_flag_ballCollectedAuto, std_msgs::Bool _out_flag_desiredVacuumCommand, std_msgs::Bool _out_flag_desiredMoveCommand #
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
    # Partial transition function call: fun3
    self.transitionFunction_idleBehaviour_fun3()
    # Partial transition function call: fun4
    self.transitionFunction_idleBehaviour_fun4()
    # Partial transition function call: fun5
    self.transitionFunction_idleBehaviour_fun5()
    # Partial transition function call: fun6
    self.transitionFunction_idleBehaviour_fun6()
    # Partial transition function call: fun7
    self.transitionFunction_idleBehaviour_fun7()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_idleBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_desiredRobotCommandTele and  not self._in_flag_desiredRobotCommandAuto:
      self.transitionFunction_idleBehaviour_fun1_0()
    elif  not self._in_flag_desiredRobotCommandTele and self._in_flag_desiredRobotCommandAuto:
      self.transitionFunction_idleBehaviour_fun1_1()
    elif  not (self._in_flag_desiredRobotCommandTele and  not self._in_flag_desiredRobotCommandAuto) and  not ( not self._in_flag_desiredRobotCommandTele and self._in_flag_desiredRobotCommandAuto):
      self.transitionFunction_idleBehaviour_fun1_2()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_desiredRobotCommandTele and  not self._in_flag_desiredRobotCommandAuto #####
  def transitionFunction_idleBehaviour_fun1_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredMoveCommand=True
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA BBBBBBBB")
    print("[CS - ballCollector] -- idleBehaviour")
    if(self.desiredRobotCommandTele.data=="stop"):
      self.desiredMoveCommand.cmd.data="stop"
      self.currentRobotSpeed.data=0
    elif(self.desiredRobotCommandTele.data=="rotate left"):
      self.desiredMoveCommand.cmd.data="rotate left"
    elif(self.desiredRobotCommandTele.data=="rotate right"):
      self.desiredMoveCommand.cmd.data="rotate right"
    else:
      self.desiredMoveCommand.cmd.data="empty"
      if(self.desiredRobotCommandTele.data=="move faster"):
        self.currentRobotSpeed.data=self.currentRobotSpeed.data+speed_delta
        if(self.currentRobotSpeed.data>200):
          self.currentRobotSpeed.data=200
      elif(self.desiredRobotCommandTele.data=="move slower"):
        self.currentRobotSpeed.data=self.currentRobotSpeed.data-speed_delta
        if(self.currentRobotSpeed.data<0):
          self.currentRobotSpeed.data=0
      elif(self.desiredRobotCommandTele.data=="move front"):
        self.desiredMoveCommand.direction.data=0
      elif(self.desiredRobotCommandTele.data=="move backwards"):
        self.desiredMoveCommand.direction.data=180
      elif(self.desiredRobotCommandTele.data=="move left"):
        self.desiredMoveCommand.direction.data=270
      elif(self.desiredRobotCommandTele.data=="move right"):
        self.desiredMoveCommand.direction.data=90   
      else:
        self._out_flag_desiredMoveCommand=False 
        print("TEST")
    self.desiredMoveCommand.desiredSpeed.data=self.currentRobotSpeed.data
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA BBBBBBBB 22222")
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not self._in_flag_desiredRobotCommandTele and self._in_flag_desiredRobotCommandAuto #####
  def transitionFunction_idleBehaviour_fun1_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredMoveCommand=True
    print("[CS - ballCollector] -- idleBehaviour")
    if(self.desiredRobotCommandAuto.data=="stop"):
      self.desiredMoveCommand.cmd.data="stop"
      self.currentRobotSpeed.data=0
    elif(self.desiredRobotCommandAuto.data=="rotate left"):
      self.desiredMoveCommand.cmd.data="rotate left"
    elif(self.desiredRobotCommandAuto.data=="rotate right"):
      self.desiredMoveCommand.cmd.data="rotate right"
    else:
      if(self.desiredRobotCommandAuto.data=="move faster"):
        self.currentRobotSpeed.data=self.currentRobotSpeed.data+speed_delta
        if(self.currentRobotSpeed.data>200):
          self.currentRobotSpeed.data=200
      elif(self.desiredRobotCommandAuto.data=="move slower"):
        self.currentRobotSpeed.data=self.currentRobotSpeed.data-speed_delta
        if(self.currentRobotSpeed.data<0):
          self.currentRobotSpeed.data=0
      else:
        self.desiredMoveCommand.cmd.data="empty"
        if(self.desiredRobotCommandAuto.data=="move front"):
          self.desiredMoveCommand.direction.data=0
        elif(self.desiredRobotCommandAuto.data=="move backwards"):
          self.desiredMoveCommand.direction.data=180
        elif(self.desiredRobotCommandAuto.data=="move left"):
          self.desiredMoveCommand.direction.data=270
        elif(self.desiredRobotCommandAuto.data=="move right"):
          self.desiredMoveCommand.direction.data=90    
        else:
          self._out_flag_desiredMoveCommand=False
    self.desiredMoveCommand.desiredSpeed.data=self.currentRobotSpeed.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_2 based on input buffers  not (self._in_flag_desiredRobotCommandTele and  not self._in_flag_desiredRobotCommandAuto) and  not ( not self._in_flag_desiredRobotCommandTele and self._in_flag_desiredRobotCommandAuto) #####
  def transitionFunction_idleBehaviour_fun1_2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_desiredRobotCommandAuto and  not self._in_flag_desiredRobotCommandTele:
      self.transitionFunction_idleBehaviour_fun2_0()
    elif  not self._in_flag_desiredRobotCommandAuto and self._in_flag_desiredRobotCommandTele:
      self.transitionFunction_idleBehaviour_fun2_1()
    elif  not (self._in_flag_desiredRobotCommandAuto and  not self._in_flag_desiredRobotCommandTele) and  not ( not self._in_flag_desiredRobotCommandAuto and self._in_flag_desiredRobotCommandTele):
      self.transitionFunction_idleBehaviour_fun2_2()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_desiredRobotCommandAuto and  not self._in_flag_desiredRobotCommandTele #####
  def transitionFunction_idleBehaviour_fun2_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredVacuumCommand=True
    print("[CS - ballCollector] -- idleBehaviour")
    if(self.desiredRobotCommandAuto.data=="vacuum turn on"):
      self.desiredVacuumCommand.data=vacuum_max
    elif(self.desiredRobotCommandAuto.data=="vacuum off"):
      self.desiredVacuumCommand.data=0
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not self._in_flag_desiredRobotCommandAuto and self._in_flag_desiredRobotCommandTele #####
  def transitionFunction_idleBehaviour_fun2_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredVacuumCommand=True
    print("[CS - ballCollector] -- idleBehaviour")
    if(self.desiredRobotCommandTele.data=="vacuum turn on"):
      self.desiredVacuumCommand.data=vacuum_max
    elif(self.desiredRobotCommandTele.data=="vacuum off"):
      self.desiredVacuumCommand.data=0
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA BBBBBBBB 22222 3333")
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_2 based on input buffers  not (self._in_flag_desiredRobotCommandAuto and  not self._in_flag_desiredRobotCommandTele) and  not ( not self._in_flag_desiredRobotCommandAuto and self._in_flag_desiredRobotCommandTele) #####
  def transitionFunction_idleBehaviour_fun2_2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun2_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun3(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun3")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_sensorInfo:
      self.transitionFunction_idleBehaviour_fun3_0()
    elif  not (self._in_flag_sensorInfo):
      self.transitionFunction_idleBehaviour_fun3_1()
    pass

  ##### Partial transition function: fun3_0 based on input buffers self._in_flag_sensorInfo #####
  def transitionFunction_idleBehaviour_fun3_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun3_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredMoveCommand=True
    print("DDDDDDDDDDDDDDDDDDDDD")
    print("[CS - ballCollector] -- idleBehaviour - check sensors: sonars")
    flag=False
    compas=0
    # check sonar nr 1 (North)
    if(self.sensorInfo.sonar_1.data < minDistanceToObstacle):
      # too close to obstacle
      # change direction and change speed
      flag=True
      compas=180  # move to south
    elif(self.sensorInfo.sonar_2.data < minDistanceToObstacle):
      # EAST
      # change direction and change speed
      flag=True
      compas=270  # move to west
    elif(self.sensorInfo.sonar_3.data < minDistanceToObstacle):
      # SOUTH
      # change direction and change speed
      flag=True
      compas=0 # move to north
    elif(self.sensorInfo.sonar_4.data < minDistanceToObstacle):
      # WEST
      # change direction and change speed
      flag=True
      compas=90 # move to east
    if(flag):
      self.currentRobotSpeed.data=0
      self.desiredMoveCommand.direction.data=compas
      self.desiredMoveCommand.desiredSpeed.data=self.currentRobotSpeed.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun3_1 based on input buffers  not (self._in_flag_sensorInfo) #####
  def transitionFunction_idleBehaviour_fun3_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun3_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun4(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun4")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_sensorInfo:
      self.transitionFunction_idleBehaviour_fun4_0()
    elif  not (self._in_flag_sensorInfo):
      self.transitionFunction_idleBehaviour_fun4_1()
    pass

  ##### Partial transition function: fun4_0 based on input buffers self._in_flag_sensorInfo #####
  def transitionFunction_idleBehaviour_fun4_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun4_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_obstacleDetectedTele=True
    self._out_flag_obstacleDetectedAuto=True
    self._out_flag_ballCollectedAuto=True
    self._out_flag_ballCollectedTele=True
    self.obstacleDetectedTele.sonar_1.data=self.sensorInfo.sonar_1.data
    self.obstacleDetectedTele.sonar_2.data=self.sensorInfo.sonar_2.data
    self.obstacleDetectedTele.sonar_3.data=self.sensorInfo.sonar_3.data
    self.obstacleDetectedTele.sonar_4.data=self.sensorInfo.sonar_4.data
    self.obstacleDetectedAuto.sonar_1.data=self.sensorInfo.sonar_1.data
    self.obstacleDetectedAuto.sonar_2.data=self.sensorInfo.sonar_2.data
    self.obstacleDetectedAuto.sonar_3.data=self.sensorInfo.sonar_3.data
    self.obstacleDetectedAuto.sonar_4.data=self.sensorInfo.sonar_4.data
    self.ballCollectedAuto.data=self.sensorInfo.inlet.data
    self.ballCollectedTele.data=self.sensorInfo.inlet.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun4_1 based on input buffers  not (self._in_flag_sensorInfo) #####
  def transitionFunction_idleBehaviour_fun4_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun4_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun5(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun5")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_cameraInfo:
      self.transitionFunction_idleBehaviour_fun5_0()
    elif  not (self._in_flag_cameraInfo):
      self.transitionFunction_idleBehaviour_fun5_1()
    pass

  ##### Partial transition function: fun5_0 based on input buffers self._in_flag_cameraInfo #####
  def transitionFunction_idleBehaviour_fun5_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun5_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_ballInfoAuto=True
    self._out_flag_ballInfoTele=True
    self.ballInfoAuto=self.cameraInfo
    self.ballInfoTele=self.cameraInfo
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun5_1 based on input buffers  not (self._in_flag_cameraInfo) #####
  def transitionFunction_idleBehaviour_fun5_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun5_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun6(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun6")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_desiredRobotSpeedAuto:
      self.transitionFunction_idleBehaviour_fun6_0()
    elif  not (self._in_flag_desiredRobotSpeedAuto):
      self.transitionFunction_idleBehaviour_fun6_1()
    pass

  ##### Partial transition function: fun6_0 based on input buffers self._in_flag_desiredRobotSpeedAuto #####
  def transitionFunction_idleBehaviour_fun6_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun6_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredMoveCommand=True
    self.currentRobotSpeed.data=self.desiredRobotSpeedAuto.data
    self.desiredMoveCommand.desiredSpeed.data=self.currentRobotSpeed.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun6_1 based on input buffers  not (self._in_flag_desiredRobotSpeedAuto) #####
  def transitionFunction_idleBehaviour_fun6_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun6_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviour_fun7(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun7")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_desiredVaccumSpeedAuto:
      self.transitionFunction_idleBehaviour_fun7_0()
    elif  not (self._in_flag_desiredVaccumSpeedAuto):
      self.transitionFunction_idleBehaviour_fun7_1()
    pass

  ##### Partial transition function: fun7_0 based on input buffers self._in_flag_desiredVaccumSpeedAuto #####
  def transitionFunction_idleBehaviour_fun7_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun7_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_desiredVacuumCommand=True
    print("[CS - ballCollector] -- idleBehaviour - fun7")
    self.desiredVacuumCommand.data=self.desiredVaccumSpeedAuto.data
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun7_1 based on input buffers  not (self._in_flag_desiredVaccumSpeedAuto) #####
  def transitionFunction_idleBehaviour_fun7_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun7_1")
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
    self._in_flag_desiredRobotCommandTele=False
    self._in_flag_desiredRobotCommandAuto=False
    self._in_flag_cameraInfo=False
    self._in_flag_sensorInfo=False
    self._in_flag_desiredRobotSpeedAuto=False
    self._in_flag_desiredVaccumSpeedAuto=False
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
    # check if output buffer obstacleDetectedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_obstacleDetectedAuto ):
      # send data from output buffer obstacleDetectedAuto
      # Buffer obstacleDetectedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_obstacleDetectedAuto.publish(self.obstacleDetectedAuto) # sending data from output buffer obstacleDetectedAuto #
      # indicate that data was sent and now the output buffer obstacleDetectedAuto is empty
      self._out_flag_obstacleDetectedAuto=False
    # check if output buffer obstacleDetectedTele has new data - i.e. is ready to send new data
    if( self._out_flag_obstacleDetectedTele ):
      # send data from output buffer obstacleDetectedTele
      # Buffer obstacleDetectedTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_obstacleDetectedTele.publish(self.obstacleDetectedTele) # sending data from output buffer obstacleDetectedTele #
      # indicate that data was sent and now the output buffer obstacleDetectedTele is empty
      self._out_flag_obstacleDetectedTele=False
    # check if output buffer ballInfoAuto has new data - i.e. is ready to send new data
    if( self._out_flag_ballInfoAuto ):
      # send data from output buffer ballInfoAuto
      # Buffer ballInfoAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballInfoAuto.publish(self.ballInfoAuto) # sending data from output buffer ballInfoAuto #
      # indicate that data was sent and now the output buffer ballInfoAuto is empty
      self._out_flag_ballInfoAuto=False
    # check if output buffer ballInfoTele has new data - i.e. is ready to send new data
    if( self._out_flag_ballInfoTele ):
      # send data from output buffer ballInfoTele
      # Buffer ballInfoTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballInfoTele.publish(self.ballInfoTele) # sending data from output buffer ballInfoTele #
      # indicate that data was sent and now the output buffer ballInfoTele is empty
      self._out_flag_ballInfoTele=False
    # check if output buffer ballCollectedTele has new data - i.e. is ready to send new data
    if( self._out_flag_ballCollectedTele ):
      # send data from output buffer ballCollectedTele
      # Buffer ballCollectedTele - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballCollectedTele.publish(self.ballCollectedTele) # sending data from output buffer ballCollectedTele #
      # indicate that data was sent and now the output buffer ballCollectedTele is empty
      self._out_flag_ballCollectedTele=False
    # check if output buffer ballCollectedAuto has new data - i.e. is ready to send new data
    if( self._out_flag_ballCollectedAuto ):
      # send data from output buffer ballCollectedAuto
      # Buffer ballCollectedAuto - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_ballCollectedAuto.publish(self.ballCollectedAuto) # sending data from output buffer ballCollectedAuto #
      # indicate that data was sent and now the output buffer ballCollectedAuto is empty
      self._out_flag_ballCollectedAuto=False
    # check if output buffer desiredVacuumCommand has new data - i.e. is ready to send new data
    if( self._out_flag_desiredVacuumCommand ):
      # send data from output buffer desiredVacuumCommand
      # Buffer desiredVacuumCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredVacuumCommand.publish(self.desiredVacuumCommand) # sending data from output buffer desiredVacuumCommand #
      # indicate that data was sent and now the output buffer desiredVacuumCommand is empty
      self._out_flag_desiredVacuumCommand=False
    # check if output buffer desiredMoveCommand has new data - i.e. is ready to send new data
    if( self._out_flag_desiredMoveCommand ):
      # send data from output buffer desiredMoveCommand
      # Buffer desiredMoveCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_desiredMoveCommand.publish(self.desiredMoveCommand) # sending data from output buffer desiredMoveCommand #
      # indicate that data was sent and now the output buffer desiredMoveCommand is empty
      self._out_flag_desiredMoveCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer desiredRobotCommandTele - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotCommandTele
    # Buffer desiredRobotCommandAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotCommandAuto
    # Buffer cameraInfo - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer cameraInfo
    # Buffer sensorInfo - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer sensorInfo
    # Buffer detectedBalls - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer detectedBalls
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
    # Buffer desiredRobotSpeedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredRobotSpeedAuto
    # Buffer desiredVaccumSpeedAuto - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer desiredVaccumSpeedAuto
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

