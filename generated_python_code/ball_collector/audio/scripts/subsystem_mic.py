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
from auxiliary_subsystem_mic import *

# Temporary definitions #
IS_LOG = False # Flag determining if logs are shown in the terminal #
IS_PRINT = True # Flag indicating if debug information for developer are shown in the terminal #
class mic:
  ##### Subsystem mic constructor #####
  def __init__(self):
    self.log("__init__ function")
    rospy.init_node("mic")
    self._subsystemName="mic"
    self._subsystemFrequency=5;
    self._currentSubsystemBehaviour="Behaviour_idleBehaviour";
    self._subsystemIterations=0
    self._behaviourIterations=0
    self.initialiseCommunicationModel()
    self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
    # initialize all input flags
    # initialize all output flags
    self._out_flag_veCSRecognisedCommand=False
    pass

  ##### Start subsystem #####
  def startSubsystem(self):
    self.log("startSubsystem")
    try:
      while self.auxiliaryFunctions.isSubsystemOK():
        ''' Execute behaviour associated with _currentSubsystemBehaviour -- choose appropriate state based on _currentSubsystemBehaviour '''
        if self._currentSubsystemBehaviour=="Behaviour_idleBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_idleBehaviour")
          self.subsystemBehaviour_idleBehaviour()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_speechRecognizeBehaviour":
          self.log("_currentSubsystemBehaviour==Behaviour_speechRecognizeBehaviour")
          self.subsystemBehaviour_speechRecognizeBehaviour()
          continue
    except Exception as e:
      print e
      self.error("Error found in function startSubsystem -- file subsystem_mic.py!")
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
    # Buffer name=veCSRecognisedCommand - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_veCSRecognisedCommand=rospy.Publisher("veCSRecognisedCommand", String, queue_size=CHANNEL_SIZE)
    pass

  ##### Initialise send channel for diagnostics #####
  def initialiseSendChannelForDiagnostics(self):
    self.log("initialiseSendChannelForDiagnostics")
    self._vectorOfSenderDiagnostics=[]
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_currentSubsystemBehaviour', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_subsystemFrequency', Float64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_subsystemName', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_subsystemIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_behaviourIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('mic/_out_flag_veCSRecognisedCommand', Bool, queue_size=CHANNEL_SIZE))
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
    self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemBehaviour)
    self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
    self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
    self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
    self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
    ###### internal state #####
    if(5 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self._out_flag_veCSRecognisedCommand)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour idleBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_idleBehaviour(self): # std_msgs::Bool _out_flag_veCSRecognisedCommand #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # std_msgs::Bool _out_flag_veCSRecognisedCommand #
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
    global responses
    global streaming_config
    global requests
    global client
    language_code = 'en-US'  # a BCP-47 language tag
    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)  
    self.veCSRecognisedCommand=String("empty")
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
    # check if output buffer veCSRecognisedCommand has new data - i.e. is ready to send new data
    if( self._out_flag_veCSRecognisedCommand ):
      # send data from output buffer veCSRecognisedCommand
      # Buffer veCSRecognisedCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_veCSRecognisedCommand.publish(self.veCSRecognisedCommand) # sending data from output buffer veCSRecognisedCommand #
      # indicate that data was sent and now the output buffer veCSRecognisedCommand is empty
      self._out_flag_veCSRecognisedCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
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

  ##### Behaviour speechRecognizeBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_speechRecognizeBehaviour(self): # std_msgs::Bool _out_flag_veCSRecognisedCommand #
    self.log("[Behaviour speechRecognizeBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_speechRecognizeBehaviour(self): # std_msgs::Bool _out_flag_veCSRecognisedCommand #
    self.log("[Behaviour speechRecognizeBehaviour] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_speechRecognizeBehaviour(self): 
    self.log("[Behaviour speechRecognizeBehaviour] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - speechRecognizeBehaviour consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_speechRecognizeBehaviour_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_speechRecognizeBehaviour_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_speechRecognizeBehaviour_fun1(self): 
    self.log("[Behaviour speechRecognizeBehaviour] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - speechRecognizeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_speechRecognizeBehaviour_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_speechRecognizeBehaviour_fun1_0(self): 
    self.log("[Behaviour speechRecognizeBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - speechRecognizeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_veCSRecognisedCommand=True
    try:
      with MicrophoneStream(RATE, CHUNK) as stream:
          audio_generator = stream.generator()
          requests = (types.StreamingRecognizeRequest(audio_content=content)
                      for content in audio_generator)
          print("A\n")
          responses = client.streaming_recognize(streaming_config, requests)
          command=listen_print_loop(responses)
          print(command)
          self.veCSRecognisedCommand.data=str(command)
          print('Received command=',self.veCSRecognisedCommand.data)
    except:
      print("Error - in audio agent - virtual receptor - mic!")      
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_speechRecognizeBehaviour_set_buffer_flags_function(self): 
    self.log("[Behaviour speechRecognizeBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - speechRecognizeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_speechRecognizeBehaviour_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_speechRecognizeBehaviour_set_buffer_flags_function_0(self): 
    self.log("[Behaviour speechRecognizeBehaviour] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - speechRecognizeBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_speechRecognizeBehaviour(self):
    self.log("[Behaviour speechRecognizeBehaviour] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer veCSRecognisedCommand has new data - i.e. is ready to send new data
    if( self._out_flag_veCSRecognisedCommand ):
      # send data from output buffer veCSRecognisedCommand
      # Buffer veCSRecognisedCommand - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_veCSRecognisedCommand.publish(self.veCSRecognisedCommand) # sending data from output buffer veCSRecognisedCommand #
      # indicate that data was sent and now the output buffer veCSRecognisedCommand is empty
      self._out_flag_veCSRecognisedCommand=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_speechRecognizeBehaviour(self):
    self.log("[Behaviour speechRecognizeBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour speechRecognizeBehaviour #####
  def executeBehaviour_speechRecognizeBehaviour(self):
    self.log("[Behaviour speechRecognizeBehaviour] -- Executing speechRecognizeBehaviour Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour speechRecognizeBehaviour #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_speechRecognizeBehaviour()
      # Sends data! #
      self.sendData_speechRecognizeBehaviour()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_speechRecognizeBehaviour()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_speechRecognizeBehaviour() or self.errorCondition_speechRecognizeBehaviour()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass


  ##### Definition of functions responsible for switching subsystem mic between states : Behaviour_idleBehaviour Behaviour_speechRecognizeBehaviour  #####
  # Behaviour idleBehaviour: #
  def subsystemBehaviour_idleBehaviour(self):
    self.log("subsystemBehaviour_idleBehaviour")
    # Executing behaviour idleBehaviour #
    self.executeBehaviour_idleBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour idleBehaviour: switching to behaviour speechRecognizeBehaviour #
    if self.initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_speechRecognizeBehaviour():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_speechRecognizeBehaviour"

    pass


  # Behaviour speechRecognizeBehaviour: #
  def subsystemBehaviour_speechRecognizeBehaviour(self):
    self.log("subsystemBehaviour_speechRecognizeBehaviour")
    # Executing behaviour speechRecognizeBehaviour #
    self.executeBehaviour_speechRecognizeBehaviour()
    # Behaviour has been terminated #
    pass


  ##### Initial condition for behaviour idleBehaviour: switching to behaviour speechRecognizeBehaviour #####
  def initialCondition_From_Behaviour_idleBehaviour_To_Behaviour_speechRecognizeBehaviour(self):
    # Initial condition specified by user #
    return  True 



  ##### Function indicating basic log/debug message #####
  def log(self, str):
    if(IS_LOG):
      rospy.loginfo("[SUBSYSTEM mic] -- LOG -- "+str)
    if(IS_PRINT):
      print "[SUBSYSTEM mic] -- LOG -- " + str
    pass

  ##### Function indicating basic error message #####
  def error(self, str):
    sys.stderr.write("[SUBSYSTEM mic] -- ERROR -- " + str)
    if(IS_LOG):
      rospy.loginfo("[SUBSYSTEM mic] -- ERROR -- "+str)
      sys.exit()
    pass

##### MAIN FUNCTION FOR SUBSYSTEM mic #####
if __name__ == '__main__':
  try:
    subsystem_mic = mic()
    subsystem_mic.startSubsystem()
  except rospy.ROSInterruptException:
    pass

