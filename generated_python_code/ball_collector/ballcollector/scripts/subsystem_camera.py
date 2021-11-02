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
from auxiliary_subsystem_camera import *

# Temporary definitions #
IS_LOG = False # Flag determining if logs are shown in the terminal #
IS_PRINT = True # Flag indicating if debug information for developer are shown in the terminal #
class camera:
  ##### Subsystem camera constructor #####
  def __init__(self):
    self.log("__init__ function")
    rospy.init_node("camera")
    self._subsystemName="camera"
    self._subsystemFrequency=10;
    self._currentSubsystemBehaviour="Behaviour_initBehaviour";
    self._subsystemIterations=0
    self._behaviourIterations=0
    self.initialiseCommunicationModel()
    self.auxiliaryFunctions = AuxiliaryFunctions(self._subsystemFrequency)
    # initialize all input flags
    # initialize all output flags
    self._out_flag_cameraInfo=False
    self._out_flag_detectedBalls=False
    self._out_flag_rpiCamera=False
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
      self.error("Error found in function startSubsystem -- file subsystem_camera.py!")
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
    # Buffer name=cameraInfo - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_cameraInfo=rospy.Publisher("cameraInfoChannel", CameraMessage, queue_size=CHANNEL_SIZE)
    # Buffer name=detectedBalls - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_detectedBalls=rospy.Publisher("detectedBallsChannel", Image, queue_size=CHANNEL_SIZE)
    # Buffer name=rpiCamera - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_rpiCamera=rospy.Publisher("rpiCameraChannel", Image, queue_size=CHANNEL_SIZE)
    pass

  ##### Initialise send channel for diagnostics #####
  def initialiseSendChannelForDiagnostics(self):
    self.log("initialiseSendChannelForDiagnostics")
    self._vectorOfSenderDiagnostics=[]
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_currentSubsystemBehaviour', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_subsystemFrequency', Float64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_subsystemName', String, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_subsystemIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_behaviourIterations', Int64, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_out_flag_cameraInfo', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_out_flag_detectedBalls', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('camera/_out_flag_rpiCamera', Bool, queue_size=CHANNEL_SIZE))
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
    if(7 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self._out_flag_cameraInfo)
      self._vectorOfSenderDiagnostics[6].publish(self._out_flag_detectedBalls)
      self._vectorOfSenderDiagnostics[7].publish(self._out_flag_rpiCamera)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # std_msgs::Bool _out_flag_cameraInfo, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_rpiCamera #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # std_msgs::Bool _out_flag_cameraInfo, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_rpiCamera #
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
    print("[VR - camera] -- initBehaviour")
    # Change the current directory  
    # get user path
    home = os.path.expanduser("~")
    # set path where the image will be saved:
    directory=home+"/git/robot_pi/ros/generated_python/data/img"
    print(directory)
    # set directory as a current path
    os.chdir(directory) 
    self.detectedBalls=Image()
    self.rpiCamera=Image()
    self.camera=PiCamera()
    self.camera.resolution=(IMAGE_X_SIZE,IMAGE_Y_SIZE)
    self.camera.framerate=32
    self.cameraInfo=CameraMessage()
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
    # check if output buffer cameraInfo has new data - i.e. is ready to send new data
    if( self._out_flag_cameraInfo ):
      # send data from output buffer cameraInfo
      # Buffer cameraInfo - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_cameraInfo.publish(self.cameraInfo) # sending data from output buffer cameraInfo #
      # indicate that data was sent and now the output buffer cameraInfo is empty
      self._out_flag_cameraInfo=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer rpiCamera has new data - i.e. is ready to send new data
    if( self._out_flag_rpiCamera ):
      # send data from output buffer rpiCamera
      # Buffer rpiCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_rpiCamera.publish(self.rpiCamera) # sending data from output buffer rpiCamera #
      # indicate that data was sent and now the output buffer rpiCamera is empty
      self._out_flag_rpiCamera=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
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
  def terminalCondition_idleBehaviour(self): # std_msgs::Bool _out_flag_cameraInfo, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_rpiCamera #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # std_msgs::Bool _out_flag_cameraInfo, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_rpiCamera #
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
    self._out_flag_cameraInfo=True
    self._out_flag_detectedBalls=True
    self._out_flag_rpiCamera=True
    print("[VR - camera] -- idleBehaviour")
    print("A 1")
    rawCapture = PiRGBArray(self.camera,size=(IMAGE_X_SIZE,IMAGE_Y_SIZE))
    print("A 2")
    time.sleep(0.1)
    self.camera.capture(rawCapture, format="bgr")
    print("A 3")
    image=rawCapture.array
    print("A 4")
    # resize image
    dim = (IMAGE_X_SIZE, IMAGE_Y_SIZE)
    # img=cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    img=cv2.resize(image, dim)
    self.rpiCamera=bridge.cv2_to_imgmsg(img,"bgr8")
    return
    ############################# OLD VERSION 
    try:
      print("B")
      print("C")
      hsvLow = tuple(rospy.get_param("/hsvLowPi"))
      hsvHigh = tuple(rospy.get_param("/hsvHighPi"))
      minCirclePi =rospy.get_param("/minCirclePi")
      param1Pi = rospy.get_param("/param1Pi")
      param2Pi =rospy.get_param("/param2Pi")
      minRadiusPi =rospy.get_param("/minRadiusPi")
      maxRadiusPi =rospy.get_param("/maxRadiusPi")
      print("D")
      print("E")
    except rospy.ROSException:
      print("F")
      print("Could not get param name")
      self._out_flag_detectedBalls=False
      self._out_flag_rpiCamera=False
      self._out_flag_cameraInfo=False
      return
    except Exception as e:
      print("F")
      print("Could not get param name")
      self._out_flag_detectedBalls=False
      self._out_flag_rpiCamera=False
      self._out_flag_cameraInfo=False
      hsvLow = (0, 39, 167)
      hsvHigh = (179, 130,255)
      minCirclePi =32
      param1Pi = 35
      param2Pi = 15
      minRadiusPi =14
      maxRadiusPi =36
      return
    print("H")
    print("J")
    # blur image
    img2 = cv2.medianBlur(img,5)
    print("K")
    # change image to hsv space
    hsv = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
    # set HSV mask
    mask = cv2.inRange(hsv,hsvLow, hsvHigh)
    # erode image
    mask = cv2.erode(mask, None, iterations=3)
    # dilate image
    mask = cv2.dilate(mask, None, iterations=3)
    # bitwise and
    img2=cv2.bitwise_and(img2,img2,mask=mask)
    # change to gray scale:
    cimg = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # find cirles:
    circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,minCirclePi, param1=param1Pi,param2=param2Pi ,minRadius=minRadiusPi,maxRadius=maxRadiusPi)
    flag=False
    # check if balls were found:
    if circles is not None:
      circles = np.uint16(np.around(circles))
      self.cameraInfo.ballVisible.data=True
      ( self.cameraInfo.ballPosition.x, self.cameraInfo.ballPosition.y , self.cameraInfo.ballPosition.z) = getBestBall(circles)
      img2=drawCircle(img2,circles)
      img2=drawSelectedCircle(img2, self.cameraInfo.ballPosition.x, self.cameraInfo.ballPosition.y , self.cameraInfo.ballPosition.z)
    else:
      self.cameraInfo.ballVisible.data=False
    # set new value within detectedBalls
    self.detectedBalls=bridge.cv2_to_imgmsg(img2,"bgr8")
    self.rpiCamera=bridge.cv2_to_imgmsg(img,"bgr8")
    # save image to file
    cv2.imwrite('ballDetected_new.jpg', img)
    rawCapture.truncate()
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
    # check if output buffer cameraInfo has new data - i.e. is ready to send new data
    if( self._out_flag_cameraInfo ):
      # send data from output buffer cameraInfo
      # Buffer cameraInfo - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_cameraInfo.publish(self.cameraInfo) # sending data from output buffer cameraInfo #
      # indicate that data was sent and now the output buffer cameraInfo is empty
      self._out_flag_cameraInfo=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer rpiCamera has new data - i.e. is ready to send new data
    if( self._out_flag_rpiCamera ):
      # send data from output buffer rpiCamera
      # Buffer rpiCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_rpiCamera.publish(self.rpiCamera) # sending data from output buffer rpiCamera #
      # indicate that data was sent and now the output buffer rpiCamera is empty
      self._out_flag_rpiCamera=False
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


  ##### Definition of functions responsible for switching subsystem camera between states : Behaviour_initBehaviour Behaviour_idleBehaviour  #####
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
      rospy.loginfo("[SUBSYSTEM camera] -- LOG -- "+str)
    if(IS_PRINT):
      print "[SUBSYSTEM camera] -- LOG -- " + str
    pass

  ##### Function indicating basic error message #####
  def error(self, str):
    sys.stderr.write("[SUBSYSTEM camera] -- ERROR -- " + str)
    if(IS_LOG):
      rospy.loginfo("[SUBSYSTEM camera] -- ERROR -- "+str)
      sys.exit()
    pass

##### MAIN FUNCTION FOR SUBSYSTEM camera #####
if __name__ == '__main__':
  try:
    subsystem_camera = camera()
    subsystem_camera.startSubsystem()
  except rospy.ROSInterruptException:
    pass

