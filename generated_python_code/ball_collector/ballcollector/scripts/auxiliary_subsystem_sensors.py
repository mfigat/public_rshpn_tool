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
import serial
import time
import string
from ballcollector.msg import SensorMessage
# sonar data
sonar_distance_1=0
sonar_distance_2=0
sonar_distance_3=0
sonar_distance_4=0
# current RPM
rpm1=0
rpm2=0
rpm3=0
rpm4=0
# desired PWM
pwm1=0
pwm2=0
pwm3=0
pwm4=0
# encoder readings:
pulses1=0
pulses2=0
pulses3=0
pulses4=0
is_inlet_sensor_triggered=False
MINIMAL_DATA_SIZE = 134
# read data concerning motors
def readMotorData(str):
  global rpm1, pwm1, pulses1
  global rpm2, pwm2, pulses2
  global rpm3, pwm3, pulses3
  global rpm4, pwm4, pulses4
  # motor 1
  (rpm1, pwm1, pulses1)=getMotorData(1, str)
  # motor 2
  (rpm2, pwm2, pulses2)=getMotorData(2, str)
  # motor 3
  (rpm3, pwm3, pulses3)=getMotorData(3, str)
  # motor 4
  (rpm4, pwm4, pulses4)=getMotorData(4, str)
# get data concerning specific motor
def getMotorData(motorNr, strTmp):
  # motor RPM
  rpm=getComponent("RPM_"+str(motorNr),strTmp)
  # motor PWM 
  pwm=getComponent("PWM_"+str(motorNr),strTmp)
  # motor PULSES
  pulses=getComponent("PULSES_"+str(motorNr),strTmp)
  return (rpm, pwm, pulses)
# get data from sonars
def getSonarData(strTmp):
  global sonar_distance_1
  global sonar_distance_2
  global sonar_distance_3
  global sonar_distance_4
  sonar_distance_1=getComponent("SONAR_1", strTmp)
  sonar_distance_2=getComponent("SONAR_2", strTmp)
  sonar_distance_3=getComponent("SONAR_3", strTmp)
  sonar_distance_4=getComponent("SONAR_4", strTmp)
# get data from component
def getComponent(componentName, strTmp):
  begin=strTmp.find(componentName+"=")
  begin=strTmp.find("=",begin)+1
  end=strTmp.find(":",begin)
  return strTmp[begin:end]
# Read inlet sensor data from Arduino
def readInletSensorData(strTmp):
  global is_inlet_sensor_triggered
  is_inlet_sensor_triggered=getComponent("INLET", strTmp)
def printRobotStatus():
  print("#######################")
  print("Motor 1", rpm1, pwm1, pulses1)
  print("Motor 2", rpm2, pwm2, pulses2)
  print("Motor 3", rpm3, pwm3, pulses3)
  print("Motor 4", rpm4, pwm4, pulses4)
  print("Sonar 1 distance",sonar_distance_1)
  print("Sonar 2 distance",sonar_distance_2)
  print("Sonar 3 distance",sonar_distance_3)
  print("Sonar 4 distance",sonar_distance_4)
  print("Inlet sensor flag",is_inlet_sensor_triggered)
  print("#######################")
def getSensorMessage():
  sensorMsg=SensorMessage()
  # sonar data
  sensorMsg.sonar_1.data=int(sonar_distance_1)
  sensorMsg.sonar_2.data=int(sonar_distance_2)
  sensorMsg.sonar_3.data=int(sonar_distance_3)
  sensorMsg.sonar_4.data=int(sonar_distance_4)
  # inlet sensor
  if(is_inlet_sensor_triggered=='1'):
    sensorMsg.inlet.data=False
  else:
    sensorMsg.inlet.data=True
  print("is_inlet_sensor_triggered=",is_inlet_sensor_triggered)
  print("sensorMsg.inlet.data=",sensorMsg.inlet.data)
  # encoder data
  sensorMsg.encoder_1.data=int(pulses1)
  sensorMsg.encoder_2.data=int(pulses2)
  sensorMsg.encoder_3.data=int(pulses3)
  sensorMsg.encoder_4.data=int(pulses4)
  # current wheels speed
  sensorMsg.rpm_1.data=float(rpm1)
  sensorMsg.rpm_2.data=float(rpm2)
  sensorMsg.rpm_3.data=float(rpm3)
  sensorMsg.rpm_4.data=float(rpm4)
  # current pwm for each motor
  sensorMsg.pwm_1.data=float(pwm1)
  sensorMsg.pwm_2.data=float(pwm2)
  sensorMsg.pwm_3.data=float(pwm3)
  sensorMsg.pwm_4.data=float(pwm4)
  return sensorMsg
# read ball collector robot status
def readRobotStatus(serial):
  # clear input buffer
  serial.flushInput()
  # read line
  str=serial.readline()
  # check the quality of data
  if(checkQualityOfData(str)):
    readMotorData(str)
    getSonarData(str)
    readInletSensorData(str)
    #printRobotStatus()
  else:
    print("DATA NOT AVAILABLE")
# initialize serial communication
def initialiseSerial():
  global ser
  ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
  ser.flush()
def checkQualityOfData(str):
  if(len(str) > MINIMAL_DATA_SIZE):
    return True
  return False

######################## -- END -- Auxiliary subsystem script ########################


