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
from auxiliary_agent_img_processing import *
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
    self._in_flag_colorImage=False
    self._in_flag_rpiCamera=False
    # initialize all output flags
    self._out_flag_intelCamera=False
    self._out_flag_detectedBalls=False
    self._out_flag_detectedBallsRpi=False
    self._out_flag_bestBallDetectedIntel=False
    self._out_flag_bestBallDetectedRpi=False
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
        if self._currentSubsystemBehaviour=="Behaviour_idleBehaviourYolo":
          self.log("_currentSubsystemBehaviour==Behaviour_idleBehaviourYolo")
          self.subsystemBehaviour_idleBehaviourYolo()
          continue
        if self._currentSubsystemBehaviour=="Behaviour_initYolo":
          self.log("_currentSubsystemBehaviour==Behaviour_initYolo")
          self.subsystemBehaviour_initYolo()
          continue
    except Exception as e:
      print e
      self.error("Error found in function startSubsystem -- file subsystem_cs.py!")
      pass

  ##### Update data for input buffer: colorImage #####
  def update_colorImage(self, data):
    self.log("update_colorImage")
    self.colorImage=data
    self._in_flag_colorImage=True
    pass

  ##### Update data for input buffer: rpiCamera #####
  def update_rpiCamera(self, data):
    self.log("update_rpiCamera")
    self.rpiCamera=data
    self._in_flag_rpiCamera=True
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
    # Buffer name=intelCamera - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_intelCamera=rospy.Publisher("intelCameraChannel", Image, queue_size=CHANNEL_SIZE)
    # Buffer name=detectedBalls - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_detectedBalls=rospy.Publisher("detectedBallsChannel", Image, queue_size=CHANNEL_SIZE)
    # Buffer name=detectedBallsRpi - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_detectedBallsRpi=rospy.Publisher("detectedBallsRpiChannel", Image, queue_size=CHANNEL_SIZE)
    # Buffer name=bestBallDetectedIntel - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_bestBallDetectedIntel=rospy.Publisher("bestBallDetectedIntelChannel", CameraMessage, queue_size=CHANNEL_SIZE)
    # Buffer name=bestBallDetectedRpi - Sender using NON-BLOCKING mode, receiver using NON-BLOCKING mode
    self._sender_bestBallDetectedRpi=rospy.Publisher("bestBallDetectedRpiChannel", CameraMessage, queue_size=CHANNEL_SIZE)
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
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_colorImage', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_in_flag_rpiCamera', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_intelCamera', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_detectedBalls', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_detectedBallsRpi', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_bestBallDetectedIntel', Bool, queue_size=CHANNEL_SIZE))
    self._vectorOfSenderDiagnostics.append(rospy.Publisher('cs/_out_flag_bestBallDetectedRpi', Bool, queue_size=CHANNEL_SIZE))
    pass

  ##### Initialise receive channel based on input buffers #####
  def initialiseReceiveChannel(self):
    self.log("initialiseReceiveChannel")
    # Buffer name=colorImage sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_colorImage=rospy.Subscriber("color/image_raw", Image, self.update_colorImage)
    # Buffer name=rpiCamera sender NON-BLOCKING mode - receiver NON-BLOCKING mode
    self._subscriber_rpiCamera=rospy.Subscriber("rpiCameraChannel", Image, self.update_rpiCamera)
    pass

  ##### Wait for all messages #####
  def waitForAllMessages(self):
    self.log("waitForAllMessages")
    #rospy.wait_for_message("", Image,  timeout=TOPIC_TIMEOUT)
    #rospy.wait_for_message("", Image,  timeout=TOPIC_TIMEOUT)
    pass

  ##### Publish on topics diagnostic data concerning the subsystem state #####
  def sendDataForDiagnostics(self):
    self._vectorOfSenderDiagnostics[0].publish(self._currentSubsystemBehaviour)
    self._vectorOfSenderDiagnostics[1].publish(self._subsystemFrequency)
    self._vectorOfSenderDiagnostics[2].publish(self._subsystemName)
    self._vectorOfSenderDiagnostics[3].publish(self._subsystemIterations)
    self._vectorOfSenderDiagnostics[4].publish(self._behaviourIterations)
    ###### internal state #####
    if(11 < len(self._vectorOfSenderDiagnostics) ):
      self._vectorOfSenderDiagnostics[5].publish(self._in_flag_colorImage)
      self._vectorOfSenderDiagnostics[6].publish(self._in_flag_rpiCamera)
      self._vectorOfSenderDiagnostics[7].publish(self._out_flag_intelCamera)
      self._vectorOfSenderDiagnostics[8].publish(self._out_flag_detectedBalls)
      self._vectorOfSenderDiagnostics[9].publish(self._out_flag_detectedBallsRpi)
      self._vectorOfSenderDiagnostics[10].publish(self._out_flag_bestBallDetectedIntel)
      self._vectorOfSenderDiagnostics[11].publish(self._out_flag_bestBallDetectedRpi)
    pass

  ##### Behaviour definitions #####

  ##### Behaviour initBehaviour #####
  ##### Terminal condition #####
  def terminalCondition_initBehaviour(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour initBehaviour] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initBehaviour(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
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
    print("[CS - img_processing] -- initBehaviour")
    # Change the current directory  
    # get user path
    home = os.path.expanduser("~")
    # set path where the image will be saved:
    directory=home+"/git/projekty/ros/generated_python/data/img"
    print(directory)
    # set directory as a current path
    os.chdir(directory) 
    # output buffer
    self.detectedBalls=Image()
    self.intelCamera=Image()
    self.bestBallDetectedRpi=CameraMessage()
    self.bestBallDetectedIntel=CameraMessage()
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
    # check if output buffer intelCamera has new data - i.e. is ready to send new data
    if( self._out_flag_intelCamera ):
      # send data from output buffer intelCamera
      # Buffer intelCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_intelCamera.publish(self.intelCamera) # sending data from output buffer intelCamera #
      # indicate that data was sent and now the output buffer intelCamera is empty
      self._out_flag_intelCamera=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer detectedBallsRpi has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBallsRpi ):
      # send data from output buffer detectedBallsRpi
      # Buffer detectedBallsRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBallsRpi.publish(self.detectedBallsRpi) # sending data from output buffer detectedBallsRpi #
      # indicate that data was sent and now the output buffer detectedBallsRpi is empty
      self._out_flag_detectedBallsRpi=False
    # check if output buffer bestBallDetectedIntel has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedIntel ):
      # send data from output buffer bestBallDetectedIntel
      # Buffer bestBallDetectedIntel - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedIntel.publish(self.bestBallDetectedIntel) # sending data from output buffer bestBallDetectedIntel #
      # indicate that data was sent and now the output buffer bestBallDetectedIntel is empty
      self._out_flag_bestBallDetectedIntel=False
    # check if output buffer bestBallDetectedRpi has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedRpi ):
      # send data from output buffer bestBallDetectedRpi
      # Buffer bestBallDetectedRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedRpi.publish(self.bestBallDetectedRpi) # sending data from output buffer bestBallDetectedRpi #
      # indicate that data was sent and now the output buffer bestBallDetectedRpi is empty
      self._out_flag_bestBallDetectedRpi=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initBehaviour(self):
    self.log("[Behaviour initBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer colorImage - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer colorImage
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
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
  def terminalCondition_idleBehaviour(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour idleBehaviour] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviour(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
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
    if self._in_flag_colorImage:
      self.transitionFunction_idleBehaviour_fun1_0()
    elif self._in_flag_colorImage:
      self.transitionFunction_idleBehaviour_fun1_1()
    elif  not (self._in_flag_colorImage) and  not (self._in_flag_colorImage):
      self.transitionFunction_idleBehaviour_fun1_2()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_colorImage #####
  def transitionFunction_idleBehaviour_fun1_0(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - img_processing] -- idleBehaviour")
    print("Received color image - colorImage")
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers self._in_flag_colorImage #####
  def transitionFunction_idleBehaviour_fun1_1(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_detectedBalls=True
    self._out_flag_intelCamera=True
    print("[CS - img_processing] -- idleBehaviour")
    print("Received both: color image and depth image")
    # convert ROS image to OpenCV image (cv::Mat)
    cvColorImage = bridge.imgmsg_to_cv2(self.colorImage, "bgr8")
    # cvDepthImage = bridge.imgmsg_to_cv2(self.depthImage, desired_encoding='passthrough')
    # set BGR and HSV constraints:
    print("A")
    try:
      print("B")
      bgrLow = tuple(rospy.get_param("/bgrLow"))
      bgrHigh = tuple(rospy.get_param("/bgrHigh"))
      print("C")
      hsvLow = tuple(rospy.get_param("/hsvLow"))
      hsvHigh = tuple(rospy.get_param("/hsvHigh"))
      minCircleIntel =rospy.get_param("/minCircleIntel")
      param1Intel = rospy.get_param("/param1Intel")
      param2Intel =rospy.get_param("/param2Intel")
      minRadiusIntel =rospy.get_param("/minRadiusIntel")
      maxRadiusIntel =rospy.get_param("/maxRadiusIntel")
      print("D")
      print("bgrLow=",bgrLow)
      print("E")
    except rospy.ROSException:
      print("F")
      print("Could not get param name")
      self._out_flag_detectedBalls=False
      return
    except Exception as e:
      print("F")
      print("Could not get param name")
      self._out_flag_detectedBalls=False
      return
    print("G")
    #bgrLow = (87, 187, 189)
    #bgrHigh = (255, 255, 255)
    #hsvLow = (13, 0, 204)
    #hsvHigh = (101, 154, 255)
    # resize image
    dim = (IMAGE_X_SIZE, IMAGE_Y_SIZE)
    img=cv2.resize(cvColorImage, dim, interpolation = cv2.INTER_AREA)
    print("H")
    # set BGR mask
    print("bgrLow=", bgrLow, (87, 187, 189))
    print("bgrHigh=", bgrHigh, (87, 187, 189))
    mask = cv2.inRange(img,bgrLow, bgrHigh)
    print("I")
    # create a new image as a bit and
    result = cv2.bitwise_and(img,img,mask = mask)
    print("J")
    # blur image
    img2 = cv2.medianBlur(result,5)
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
    #circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=8 ,minRadius=3,maxRadius=20)
    circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,minCircleIntel, param1=param1Intel,param2=param2Intel ,minRadius=minRadiusIntel,maxRadius=maxRadiusIntel)
    # check if balls were found:
    if circles is not None:
      circles = np.uint16(np.around(circles))
      # draw circles in image
      for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img2,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(img2,(i[0],i[1]),2,(0,0,255),3)
    # set new value within detectedBalls
    self.detectedBalls=bridge.cv2_to_imgmsg(img2,"bgr8")
    self.intelCamera=bridge.cv2_to_imgmsg(img,"bgr8")
    # save img to file
    cv2.imwrite('ballDetected.jpg', img2)
    #cv2.imwrite('depth.jpg', cvDepthImage)
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_2 based on input buffers  not (self._in_flag_colorImage) and  not (self._in_flag_colorImage) #####
  def transitionFunction_idleBehaviour_fun1_2(self): 
    self.log("[Behaviour idleBehaviour] -- Calculating Partial Transition Function fun1_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviour consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - img_processing] -- idleBehaviour")
    print("Nothing received")
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
    self._in_flag_colorImage=False
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
    # check if output buffer intelCamera has new data - i.e. is ready to send new data
    if( self._out_flag_intelCamera ):
      # send data from output buffer intelCamera
      # Buffer intelCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_intelCamera.publish(self.intelCamera) # sending data from output buffer intelCamera #
      # indicate that data was sent and now the output buffer intelCamera is empty
      self._out_flag_intelCamera=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer detectedBallsRpi has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBallsRpi ):
      # send data from output buffer detectedBallsRpi
      # Buffer detectedBallsRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBallsRpi.publish(self.detectedBallsRpi) # sending data from output buffer detectedBallsRpi #
      # indicate that data was sent and now the output buffer detectedBallsRpi is empty
      self._out_flag_detectedBallsRpi=False
    # check if output buffer bestBallDetectedIntel has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedIntel ):
      # send data from output buffer bestBallDetectedIntel
      # Buffer bestBallDetectedIntel - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedIntel.publish(self.bestBallDetectedIntel) # sending data from output buffer bestBallDetectedIntel #
      # indicate that data was sent and now the output buffer bestBallDetectedIntel is empty
      self._out_flag_bestBallDetectedIntel=False
    # check if output buffer bestBallDetectedRpi has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedRpi ):
      # send data from output buffer bestBallDetectedRpi
      # Buffer bestBallDetectedRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedRpi.publish(self.bestBallDetectedRpi) # sending data from output buffer bestBallDetectedRpi #
      # indicate that data was sent and now the output buffer bestBallDetectedRpi is empty
      self._out_flag_bestBallDetectedRpi=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviour(self):
    self.log("[Behaviour idleBehaviour] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer colorImage - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer colorImage
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
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

  ##### Behaviour idleBehaviourYolo #####
  ##### Terminal condition #####
  def terminalCondition_idleBehaviourYolo(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour idleBehaviourYolo] -- Checking Terminal Condition")
    return  False 
    pass

  ##### Error condition #####
  def errorCondition_idleBehaviourYolo(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour idleBehaviourYolo] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_idleBehaviourYolo(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - idleBehaviourYolo consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun2
    self.transitionFunction_idleBehaviourYolo_fun2()
    # Partial transition function call: fun1
    self.transitionFunction_idleBehaviourYolo_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_idleBehaviourYolo_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviourYolo_fun2(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun2")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_rpiCamera:
      self.transitionFunction_idleBehaviourYolo_fun2_0()
    elif  not (self._in_flag_rpiCamera):
      self.transitionFunction_idleBehaviourYolo_fun2_1()
    pass

  ##### Partial transition function: fun2_0 based on input buffers self._in_flag_rpiCamera #####
  def transitionFunction_idleBehaviourYolo_fun2_0(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun2_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_detectedBallsRpi=True
    self._out_flag_bestBallDetectedRpi=True
    print("[CS - img_processing] -- idleBehaviourYolo")
    print("Received both: color image from rpi camera")
    # convert ROS image to OpenCV image (cv::Mat)
    cvColorImage = bridge.imgmsg_to_cv2(self.rpiCamera, "bgr8")
    print("G")
    # resize image
    dim = (IMAGE_X_SIZE, IMAGE_Y_SIZE)
    img2=cv2.resize(cvColorImage, dim, interpolation = cv2.INTER_AREA)
    print("G 1")
    # ########################################
    try:
      flag=False
      home = os.path.expanduser("~")
      path=home+YOLO_PATH
      # save img2 to file
      cv2.imwrite(path+'imageYoloRpi.jpg', img2)
      # And then down here you could detect a lot more images like:
      print("path=",path+'imageYoloRpi.jpg')
      r = dn.detect(net, meta, path+'imageYoloRpi.jpg')
      if(len(r)>0):
        print("len>0, len=",len(r))
        flag=True
        bestBallX=0
        bestBallY=0
        bestBallRadius=0
        for i in r:
          print(i)
          what=i[0]
          prediction_val=i[1]
          points=i[2]
          if(what=="table tennis ball"):
            img2=drawRectangle(int(points[0])-int(points[2])/2, int(points[1])-int(points[3])/2, int(points[0])+int(points[2])/2, int(points[1])+int(points[3])/2,img2)
            img2=drawText(img2, what, prediction_val,int(points[0])-int(points[2])/2, int(points[1])-int(points[3])/2-10 )
            print("BBABBAA 1")
            # choose the closest ball
            if(int(points[1])>bestBallY):
              flag=True
              bestBallX=int(points[0])
              bestBallY=int(points[1])
              bestBallRadius=int(points[2])/2
            # draw the outer circle
            cv2.circle(img2,(int(points[0]),int(points[1])),int(points[2])/2,(0,0,255),2)
            # draw the center of the circle
            cv2.circle(img2,(int(points[0]),int(points[1])),2,(0,0,255),3)
    except Exception:
      print("Error")
    if(flag):
      self.bestBallDetectedRpi.ballVisible.data=True
      self.bestBallDetectedRpi.ballPosition.x=bestBallX   
      self.bestBallDetectedRpi.ballPosition.y=bestBallY
      self.bestBallDetectedRpi.ballPosition.z=bestBallRadius
      # draw the outer circle
      cv2.circle(img2,(bestBallX,bestBallY),bestBallRadius,(0,255,0),2)
      # draw the center of the circle
      cv2.circle(img2,(bestBallX,bestBallY),2,(0,0,255),3)
    else:
      self.bestBallDetectedRpi.ballVisible.data=False
    # ########################################
    # set new value within detectedBalls
    self.detectedBallsRpi=bridge.cv2_to_imgmsg(img2,"bgr8")
    # save img to file
    cv2.imwrite(path+'detectedImgRpi.jpg', img2)
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun2_1 based on input buffers  not (self._in_flag_rpiCamera) #####
  def transitionFunction_idleBehaviourYolo_fun2_1(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun2_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # Comment generated by RSSL compiler - transition function generated - data from input buffers was not received on time - code not specified
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviourYolo_fun1(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    if self._in_flag_colorImage:
      self.transitionFunction_idleBehaviourYolo_fun1_0()
    elif  not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ()):
      self.transitionFunction_idleBehaviourYolo_fun1_1()
    elif  not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ())):
      self.transitionFunction_idleBehaviourYolo_fun1_2()
    pass

  ##### Partial transition function: fun1_0 based on input buffers self._in_flag_colorImage #####
  def transitionFunction_idleBehaviourYolo_fun1_0(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._out_flag_bestBallDetectedIntel=True
    self._out_flag_detectedBalls=True
    self._out_flag_intelCamera=True
    print("[CS - img_processing] -- idleBehaviourYolo")
    print("Received both: color image and depth image")
    # convert ROS image to OpenCV image (cv::Mat)
    cvColorImage = bridge.imgmsg_to_cv2(self.colorImage, "bgr8")
    # cvDepthImage = bridge.imgmsg_to_cv2(self.depthImage, desired_encoding='passthrough')
    # set BGR and HSV constraints:
    try:
      print("B")
    except rospy.ROSException:
      print("Error in idleBehaviourYolo")
      self._out_flag_detectedBalls=False
      return
    except Exception as e:
      print("Error in idleBehaviourYolo")
      self._out_flag_detectedBalls=False
      return
    # resize image
    dim = (IMAGE_X_SIZE, IMAGE_Y_SIZE)
    img=cv2.resize(cvColorImage, dim, interpolation = cv2.INTER_AREA)
    img2=cv2.resize(cvColorImage, dim, interpolation = cv2.INTER_AREA)
    # ########################################
    try:
      flag=False
      home = os.path.expanduser("~")
      path=home+YOLO_PATH
      # save img to file
      cv2.imwrite(path+'imageYolo.jpg', img)
      # And then down here you could detect a lot more images like:
      print("path=",path+'imageYolo.jpg')
      r = dn.detect(net, meta, path+'imageYolo.jpg')
      if(len(r)>0):
        print("len>0, len=",len(r))
        flag=True
        bestBallX=0
        bestBallY=0
        bestBallRadius=0
        for i in r:
          print(i)
          what=i[0]
          prediction_val=i[1]
          points=i[2]
          if(what=="table tennis ball"):
            img2=drawRectangle(int(points[0])-int(points[2])/2, int(points[1])-int(points[3])/2, int(points[0])+int(points[2])/2, int(points[1])+int(points[3])/2,img2)
            img2=drawText(img2, what, prediction_val,int(points[0])-int(points[2])/2, int(points[1])-int(points[3])/2-10 )
            # choose the closest ball
            if(int(points[1])>bestBallY):
              flag=True
              bestBallX=int(points[0])
              bestBallY=int(points[1])
              bestBallRadius=int(points[2])/2
            # draw the outer circle
            cv2.circle(img2,(int(points[0]),int(points[1])),int(points[2])/2,(0,0,255),2)
            # draw the center of the circle
            cv2.circle(img2,(int(points[0]),int(points[1])),2,(0,0,255),3)
    except Exception:
      print("Error")
    if(flag):
      self.bestBallDetectedIntel.ballVisible.data=True
      self.bestBallDetectedIntel.ballPosition.x=bestBallX   
      self.bestBallDetectedIntel.ballPosition.y=bestBallY
      self.bestBallDetectedIntel.ballPosition.z=bestBallRadius
      # draw the outer circle
      cv2.circle(img2,(bestBallX,bestBallY),bestBallRadius,(0,255,0),2)
      # draw the center of the circle
      cv2.circle(img2,(bestBallX,bestBallY),2,(0,0,255),3)
    else:
      self.bestBallDetectedIntel.ballVisible.data=False
    # ########################################
    # set new value within detectedBalls
    self.detectedBalls=bridge.cv2_to_imgmsg(img2,"bgr8")
    self.intelCamera=bridge.cv2_to_imgmsg(img,"bgr8")
    # save img to file
    cv2.imwrite(path+'detectedImg.jpg', img2)
    # cv2.imwrite('depth.jpg', cvDepthImage)
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_1 based on input buffers  not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ()) #####
  def transitionFunction_idleBehaviourYolo_fun1_1(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun1_1")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - img_processing] -- idleBehaviourYolo")
    print("Nothing received")
    # End - Partial Transition Function Code
    pass

  ##### Partial transition function: fun1_2 based on input buffers  not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ( not (self._in_flag_colorImage) and  not ())) #####
  def transitionFunction_idleBehaviourYolo_fun1_2(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function fun1_2")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    print("[CS - img_processing] -- idleBehaviourYolo")
    print("Nothing received")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_idleBehaviourYolo_set_buffer_flags_function(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_idleBehaviourYolo_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_idleBehaviourYolo_set_buffer_flags_function_0(self): 
    self.log("[Behaviour idleBehaviourYolo] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - idleBehaviourYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    self._in_flag_colorImage=False
    self._in_flag_rpiCamera=False
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_idleBehaviourYolo(self):
    self.log("[Behaviour idleBehaviourYolo] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer intelCamera has new data - i.e. is ready to send new data
    if( self._out_flag_intelCamera ):
      # send data from output buffer intelCamera
      # Buffer intelCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_intelCamera.publish(self.intelCamera) # sending data from output buffer intelCamera #
      # indicate that data was sent and now the output buffer intelCamera is empty
      self._out_flag_intelCamera=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer detectedBallsRpi has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBallsRpi ):
      # send data from output buffer detectedBallsRpi
      # Buffer detectedBallsRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBallsRpi.publish(self.detectedBallsRpi) # sending data from output buffer detectedBallsRpi #
      # indicate that data was sent and now the output buffer detectedBallsRpi is empty
      self._out_flag_detectedBallsRpi=False
    # check if output buffer bestBallDetectedIntel has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedIntel ):
      # send data from output buffer bestBallDetectedIntel
      # Buffer bestBallDetectedIntel - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedIntel.publish(self.bestBallDetectedIntel) # sending data from output buffer bestBallDetectedIntel #
      # indicate that data was sent and now the output buffer bestBallDetectedIntel is empty
      self._out_flag_bestBallDetectedIntel=False
    # check if output buffer bestBallDetectedRpi has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedRpi ):
      # send data from output buffer bestBallDetectedRpi
      # Buffer bestBallDetectedRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedRpi.publish(self.bestBallDetectedRpi) # sending data from output buffer bestBallDetectedRpi #
      # indicate that data was sent and now the output buffer bestBallDetectedRpi is empty
      self._out_flag_bestBallDetectedRpi=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_idleBehaviourYolo(self):
    self.log("[Behaviour idleBehaviourYolo] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer colorImage - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer colorImage
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour idleBehaviourYolo #####
  def executeBehaviour_idleBehaviourYolo(self):
    self.log("[Behaviour idleBehaviourYolo] -- Executing idleBehaviourYolo Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour idleBehaviourYolo #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_idleBehaviourYolo()
      # Sends data! #
      self.sendData_idleBehaviourYolo()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_idleBehaviourYolo()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_idleBehaviourYolo() or self.errorCondition_idleBehaviourYolo()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass

  ##### Behaviour initYolo #####
  ##### Terminal condition #####
  def terminalCondition_initYolo(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour initYolo] -- Checking Terminal Condition")
    return  True 
    pass

  ##### Error condition #####
  def errorCondition_initYolo(self): # Image colorImage, Image rpiCamera, std_msgs::Bool _in_flag_colorImage, std_msgs::Bool _in_flag_rpiCamera, std_msgs::Bool _out_flag_intelCamera, std_msgs::Bool _out_flag_detectedBalls, std_msgs::Bool _out_flag_detectedBallsRpi, std_msgs::Bool _out_flag_bestBallDetectedIntel, std_msgs::Bool _out_flag_bestBallDetectedRpi #
    self.log("[Behaviour initYolo] -- Checking Error Condition")
    return  False 
    pass

  ##### Transition function #####
  def transitionFunction_initYolo(self): 
    self.log("[Behaviour initYolo] -- Calculating Transition Function")
    # Transition function #
    self.log("TRANSITION FUNCTION - initYolo consists of the following partial transition functions (decomposition based on output buffers)")
    # Partial transition function call: fun1
    self.transitionFunction_initYolo_fun1()
    # Partial transition function call: set_buffer_flags_function
    self.transitionFunction_initYolo_set_buffer_flags_function()
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_initYolo_fun1(self): 
    self.log("[Behaviour initYolo] -- Calculating Partial Transition Function fun1")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - initYolo consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_initYolo_fun1_0()
    pass

  ##### Partial transition function: fun1_0 based on input buffers True #####
  def transitionFunction_initYolo_fun1_0(self): 
    self.log("[Behaviour initYolo] -- Calculating Partial Transition Function fun1_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - initYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    global net
    global meta
    print("[CS - img_processing] -- initYolo")
    dn.set_gpu(0)
    home = os.path.expanduser("~")
    # set the current directory
    directory=home+YOLO_PATH
    print(directory)
    # set directory as a current path
    os.chdir(directory) 
    net = dn.load_net(directory+"cfg/yolo-tabletennisball.cfg", directory+"cfg/yolo-tabletennisball.weights", 0)
    print("ALA ma kota 2")
    meta = dn.load_meta(directory+"data/obj.data")
    print("ALA ma kota 3")
    # End - Partial Transition Function Code
    pass

  ##### Decomposition of partial transition function based on input buffers #####
  def transitionFunction_initYolo_set_buffer_flags_function(self): 
    self.log("[Behaviour initYolo] -- Calculating Partial Transition Function set_buffer_flags_function")
    # Partial Transition Function - the first layer #
    self.log("PARTIAL TRANSITION FUNCTION - FIRST LAYER - initYolo consists of the following partial transition functions (decomposition based on input buffers)")
    if True:
      self.transitionFunction_initYolo_set_buffer_flags_function_0()
    pass

  ##### Partial transition function: set_buffer_flags_function_0 based on input buffers True #####
  def transitionFunction_initYolo_set_buffer_flags_function_0(self): 
    self.log("[Behaviour initYolo] -- Calculating Partial Transition Function set_buffer_flags_function_0")
    # Partial Transition Function - the second layer #
    self.log("PARTIAL TRANSITION FUNCTION - SECOND LAYER - initYolo consists of the following partial transition functions (decomposition based on input buffers)")
    # Begin - Partial Transition Function Code
    # End - Partial Transition Function Code
    pass

  ##### End of transition function #####
  ##### Send data to other subsystems #####
  def sendData_initYolo(self):
    self.log("[Behaviour initYolo] -- Sending Data")
    # DIAGNOSTICS SEND #
    self.sendDataForDiagnostics()
    # END OF DIAGNOSTICS SEND #
    # TYPICAL SEND CALL #
    # check if output buffer intelCamera has new data - i.e. is ready to send new data
    if( self._out_flag_intelCamera ):
      # send data from output buffer intelCamera
      # Buffer intelCamera - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_intelCamera.publish(self.intelCamera) # sending data from output buffer intelCamera #
      # indicate that data was sent and now the output buffer intelCamera is empty
      self._out_flag_intelCamera=False
    # check if output buffer detectedBalls has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBalls ):
      # send data from output buffer detectedBalls
      # Buffer detectedBalls - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBalls.publish(self.detectedBalls) # sending data from output buffer detectedBalls #
      # indicate that data was sent and now the output buffer detectedBalls is empty
      self._out_flag_detectedBalls=False
    # check if output buffer detectedBallsRpi has new data - i.e. is ready to send new data
    if( self._out_flag_detectedBallsRpi ):
      # send data from output buffer detectedBallsRpi
      # Buffer detectedBallsRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_detectedBallsRpi.publish(self.detectedBallsRpi) # sending data from output buffer detectedBallsRpi #
      # indicate that data was sent and now the output buffer detectedBallsRpi is empty
      self._out_flag_detectedBallsRpi=False
    # check if output buffer bestBallDetectedIntel has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedIntel ):
      # send data from output buffer bestBallDetectedIntel
      # Buffer bestBallDetectedIntel - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedIntel.publish(self.bestBallDetectedIntel) # sending data from output buffer bestBallDetectedIntel #
      # indicate that data was sent and now the output buffer bestBallDetectedIntel is empty
      self._out_flag_bestBallDetectedIntel=False
    # check if output buffer bestBallDetectedRpi has new data - i.e. is ready to send new data
    if( self._out_flag_bestBallDetectedRpi ):
      # send data from output buffer bestBallDetectedRpi
      # Buffer bestBallDetectedRpi - NON-BLOCKING Sender mode - NON-BLOCKING Receiver mode
      self._sender_bestBallDetectedRpi.publish(self.bestBallDetectedRpi) # sending data from output buffer bestBallDetectedRpi #
      # indicate that data was sent and now the output buffer bestBallDetectedRpi is empty
      self._out_flag_bestBallDetectedRpi=False
    # END OF TYPICAL SEND CALL #

    # BEGIN OF BODY SEND CALL #
    # END OF BODY SEND CALL #
    pass

  ##### Receive data from other subsystems #####
  def receiveData_initYolo(self):
    self.log("[Behaviour initYolo] -- Receiving Data")
    # TYPICAL RECEIVE CALL #
    # Buffer colorImage - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer colorImage
    # Buffer rpiCamera - Sender using NON-BLOCKING MODE - Receiver using NON-BLOCKING MODE
    # ROS topic generated for the input buffer rpiCamera
    self.waitForAllMessages() #
    # END OF TYPICAL RECEIVE CALL #
    # BEGIN OF RECEIVE BODY CALL #
    # END OF RECEIVE BODY CALL #
    pass

  ##### Execute behaviour initYolo #####
  def executeBehaviour_initYolo(self):
    self.log("[Behaviour initYolo] -- Executing initYolo Behaviour")
    stopBehaviourIteration=False
    # Execution of a single iteration of a behaviour initYolo #
    _behaviourIterations=0
    # Starts execution! #
    while True:
      # Sleep is a method from class AuxiliaryFunctions which executes sleep from ROS #
      self.auxiliaryFunctions.sleep()
      # Calculates transition function -- output and internal buffers can only be modified by this function! #
      self.transitionFunction_initYolo()
      # Sends data! #
      self.sendData_initYolo()
      # Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations #
      self._behaviourIterations=self._behaviourIterations+1
      # Receives data! #
      self.receiveData_initYolo()
      # Check both conditions, i.e. terminal condition and error condition #
      stopBehaviourIteration = self.terminalCondition_initYolo() or self.errorCondition_initYolo()
      if stopBehaviourIteration or not self.auxiliaryFunctions.isSubsystemOK():
        '''
        Iterate within the while loop until stopBehaviourIteration is set true, i.e. one
        of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise
        subsystem must have been switched to another state or SIGINT was sent
        '''
        break
      # Stops execution! #
    pass


  ##### Definition of functions responsible for switching subsystem cs between states : Behaviour_initBehaviour Behaviour_idleBehaviour Behaviour_idleBehaviourYolo Behaviour_initYolo  #####
  # Behaviour initBehaviour: #
  def subsystemBehaviour_initBehaviour(self):
    self.log("subsystemBehaviour_initBehaviour")
    # Executing behaviour initBehaviour #
    self.executeBehaviour_initBehaviour()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour initBehaviour: switching to behaviour initYolo #
    if self.initialCondition_From_Behaviour_initBehaviour_To_Behaviour_initYolo():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_initYolo"

    pass


  # Behaviour idleBehaviour: #
  def subsystemBehaviour_idleBehaviour(self):
    self.log("subsystemBehaviour_idleBehaviour")
    # Executing behaviour idleBehaviour #
    self.executeBehaviour_idleBehaviour()
    # Behaviour has been terminated #
    pass


  # Behaviour idleBehaviourYolo: #
  def subsystemBehaviour_idleBehaviourYolo(self):
    self.log("subsystemBehaviour_idleBehaviourYolo")
    # Executing behaviour idleBehaviourYolo #
    self.executeBehaviour_idleBehaviourYolo()
    # Behaviour has been terminated #
    pass


  # Behaviour initYolo: #
  def subsystemBehaviour_initYolo(self):
    self.log("subsystemBehaviour_initYolo")
    # Executing behaviour initYolo #
    self.executeBehaviour_initYolo()
    # Behaviour has been terminated #
    # Checking initial condition for behaviour initYolo: switching to behaviour idleBehaviourYolo #
    if self.initialCondition_From_Behaviour_initYolo_To_Behaviour_idleBehaviourYolo():
      # incrementing the number determining how many times subsystem has switched between behaviours #
      self._subsystemIterations=self._subsystemIterations+1
      self._currentSubsystemBehaviour="Behaviour_idleBehaviourYolo"

    pass


  ##### Initial condition for behaviour initBehaviour: switching to behaviour initYolo #####
  def initialCondition_From_Behaviour_initBehaviour_To_Behaviour_initYolo(self):
    # Initial condition specified by user #
    return  True 


  ##### Initial condition for behaviour initYolo: switching to behaviour idleBehaviourYolo #####
  def initialCondition_From_Behaviour_initYolo_To_Behaviour_idleBehaviourYolo(self):
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

