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
from auxiliary_agent_ballcollector import *
def sendMessageToMotor(ser, moveCommand):
  print("moveCommand=",moveCommand)
  strSpeed=str(moveCommand.desiredSpeed.data)
  cmdToSend=""
  flag=False
  if(moveCommand.cmd.data=="stop"):
    ser.write(b"S")
  elif(moveCommand.cmd.data=="rotate left"):
    cmdToSend="D:-"+strSpeed+":"+strSpeed+":-"+strSpeed+":"+strSpeed+";"
    print(cmdToSend)
    flag=True
  elif(moveCommand.cmd.data=="rotate right"):
    cmdToSend="D:"+strSpeed+":-"+strSpeed+":"+strSpeed+":-"+strSpeed+";"
    print(cmdToSend)
    flag=True
  else:
    if(moveCommand.direction.data==0): # move front
      cmdToSend="D:"+strSpeed+":-"+strSpeed+":-"+strSpeed+":"+strSpeed+";"
      print(cmdToSend)
      flag=True
    elif(moveCommand.direction.data==180): # move backwards
      cmdToSend="D:-"+strSpeed+":"+strSpeed+":"+strSpeed+":-"+strSpeed+";"
      print(cmdToSend)
      flag=True
    elif(moveCommand.direction.data==270): # move left
      cmdToSend="D:"+strSpeed+":"+strSpeed+":-"+strSpeed+":-"+strSpeed+";"
      print(cmdToSend)  
      flag=True
    elif(moveCommand.direction.data==90): # move right
      cmdToSend="D:-"+strSpeed+":-"+strSpeed+":"+strSpeed+":"+strSpeed+";"
      print(cmdToSend)
      flag=True
  print("EEEEEEEEEEEEEEEE move command=",moveCommand)
  print("cmdToSend=",cmdToSend)
  try:
    if(flag and len(str(cmdToSend))>0):
      ser.write(cmdToSend.encode('UTF-8'))
  except:
    print("Error connection with Arduino")

######################## -- END -- Auxiliary subsystem script ########################


