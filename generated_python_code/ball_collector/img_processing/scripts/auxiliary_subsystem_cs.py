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
import cv2
YOLO_PATH="/git/projekty/ros/generated_python/yolo/"
global net
global meta
from decimal import *
def drawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY, image):
  start_point = (topLeftX, topLeftY) 
  end_point = (bottomRightX, bottomRightY) 
  color=(255,0,0) # BGR
  thickness=2
  return cv2.rectangle(image, start_point, end_point, color, thickness)
def drawText(img, what, prediction_val, x, y):
  font = cv2.FONT_HERSHEY_SIMPLEX 
  where = (x,y-5) 
  fontScale = 1
  color = (0, 0, 255) # color in BGR space
  thickness = 2
  getcontext().prec=2
  print(prediction_val)
  pred_test=int(prediction_val*100)
  pred_test=pred_test/100.0
  strTmp="tt_ball:" + str(pred_test)
  print(strTmp)
  return cv2.putText(img, strTmp, where, font, fontScale, color, thickness, cv2.LINE_AA)

######################## -- END -- Auxiliary subsystem script ########################


