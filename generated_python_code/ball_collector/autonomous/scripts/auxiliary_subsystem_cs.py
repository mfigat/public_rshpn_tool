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


######################## -- BEGIN -- Auxiliary subsystem script ########################
import rospy
global IMAGE_X_SIZE
global IMAGE_Y_SIZE
global minRpiInletX
global v_rot_max
global v_rot_min
global v_front_max
global v_front_min
global VACUUM_TURN_ON
global taskAgentObstacleMaxDistance
taskAgentObstacleMaxDistance=15
NORTH=0
EAST=1
SOUTH=2
WEST=3
NO_OBSTACLE=-1
IMAGE_X_SIZE=640
IMAGE_Y_SIZE=480
# center of pipe in RPi image (X coordinate)
midXPointRpi=IMAGE_X_SIZE/2 - 40
# the distance from midXPointRpi to the left and right -- if the ball is within this section, the robot moves front
minRpiInletX=100
# max speed of each wheel when robot rotates
v_rot_max=10
# min speed of each wheel when robot rotates
v_rot_min=5
# max speed of each wheel when robot moves within midXPointRpi corridor
v_front_max=10
# min speed of each wheel when robot moves within midXPointRpi corridor
v_front_min=5
# vacuum activated
VACUUM_TURN_ON=150
# vacuum deactivated
VACUUM_TURN_OFF=0
def checkIfObstacleDetected(sonars):
  # return values: -1 - no obstacle, 0 - north, 1 - right, 2 - south, 3 - left
  if(sonars.sonar_1.data<taskAgentObstacleMaxDistance):
    return NORTH
  elif(sonars.sonar_2.data<taskAgentObstacleMaxDistance):
    return EAST 
  elif(sonars.sonar_3.data<taskAgentObstacleMaxDistance):
    return SOUTH
  elif(sonars.sonar_4.data<taskAgentObstacleMaxDistance):
    return WEST
  else:
    return NO_OBSTACLE
def updateGlobalValuesFromROSParamServer():
  global IMAGE_X_SIZE
  global IMAGE_Y_SIZE
  global minRpiInletX
  global v_rot_max
  global v_rot_min
  global v_front_max
  global v_front_min
  global VACUUM_TURN_ON
  global taskAgentObstacleMaxDistance
  try:
    IMAGE_X_SIZE = rospy.get_param("/imageXSize")
    IMAGE_Y_SIZE = rospy.get_param("/imageYSize")
    minRpiInletX =rospy.get_param("/minRpiInletX")
    v_rot_max = rospy.get_param("/vRotMax")
    v_rot_min =rospy.get_param("/vRotMin")
    v_front_max =rospy.get_param("/vFrontMax")
    v_front_min =rospy.get_param("/vFrontMin")
    VACUUM_TURN_ON =rospy.get_param("/vacuumTurnOn")
    taskAgentObstacleMaxDistance =rospy.get_param("/taskAgentObstacleMaxDistance")
    print("D")
    print("E")
    print("AAAAAAAAAAAAAAAAAAAAAAAA")
    print("AAAAAAAAAAAAAAAAAAAAAAAA")
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
    IMAGE_X_SIZE = 320
    IMAGE_Y_SIZE = 240
    minRpiInletX =100
    v_rot_max = 10
    v_rot_min =5
    v_front_max =10
    v_front_min =5
    VACUUM_TURN_ON =150
    return

######################## -- END -- Auxiliary subsystem script ########################


