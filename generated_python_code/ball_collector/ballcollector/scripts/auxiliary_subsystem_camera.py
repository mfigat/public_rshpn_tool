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
from collections import deque
import numpy as np
import argparse
#import imutils
import cv2
#import cv2.cv as cv
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from cv_bridge import CvBridge
import cv2
import os # to change current directory
bridge = CvBridge()
# initialize camera
IMAGE_X_SIZE=320
IMAGE_Y_SIZE=240
def getBestBall(circles):
  maxBallX=0
  maxBallY=0
  maxBallRadius=0
  for circle in circles[0,:]:
    if(circle[2]>maxBallRadius):
      maxBallRadius=circle[2]
      maxBallX=circle[0]
      maxBallY=circle[1]
  return (maxBallX, maxBallY, maxBallRadius)
def drawCircle(img, circles):
  for circle in circles[0,:]:
    cv2.circle(img,(circle[0],circle[1]),circle[2],(0,0,255),2)
    cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)
  return img
def drawSelectedCircle(img, x,y,radius):
  cv2.circle(img,(x,y),radius,(0,255,0),4)
  cv2.circle(img,(x,y),2,(0,0,255),3)
  return img

######################## -- END -- Auxiliary subsystem script ########################


