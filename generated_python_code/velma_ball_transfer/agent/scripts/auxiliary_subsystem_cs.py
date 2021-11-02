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

# VELMA PYTHON INTERFACE IMPORTS
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

def makeWrench(lx, ly, lz, rx, ry, rz):
    return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))

IMP_LIST_SOFT_XY_HARD_Z_STIFFNESS=[makeWrench(50,50,1000,150,150,150)]
IMP_LIST_DEFAULT_STIFFNESS=[makeWrench(1000,1000,1000,150,150,150)]
IMP_TIME_LIST=[3.0]
PATH_TOLERANCE_DEFAULT = PyKDL.Twist(PyKDL.Vector(0.04, 0.04, 0.04), PyKDL.Vector(0.04, 0.04, 0.04))
PATH_TOLERANCE_DEFAULT_2 = PyKDL.Twist(PyKDL.Vector(0.4, 0.4, 0.4), PyKDL.Vector(0.4, 0.4, 0.4))
MAX_WRENCH_DEFAULT = PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))
SOFT_MAX_WRENCH_DEFAULT = PyKDL.Wrench(PyKDL.Vector(0.5,0.5,5), PyKDL.Vector(5,5,5))
WRENCH_GRIPPER_SOFT_CONSTRAINT_LIST = [	makeWrench(500,500,300,	150,150,150),
                                        makeWrench(250,250,125,	150,150,150),
                                        makeWrench(100,100,50,	150,150,150),
                                        makeWrench(50,50,25,	150,150,150)]
WRENCH_GRIPPER_DEFAULT_LIST_TIMES = [0.5, 1.0, 1.5, 2.0]
DEFAULT_TIME = [3.0]

STATE_START = -1
STATE_INITIALIZE_ROBOT_SYSTEM=1
STATE_MOVE_IN_FRONT_OF_FIRST_TABLE=2
STATE_TAKE_CUP_FROM_TABLE=3
STATE_MOVE_IN_FRONT_OF_THIRD_TABLE=4
STATE_PUT_CUP_ON_TABLE=5
STATE_MOVE_IN_FRONT_OF_SECOND_TABLE=6
STATE_OPEN_DOOR=7
STATE_TAKE_CUP_FROM_CABINET_AVOIDING_DOOR=8
STATE_MOVE_IN_FRONT_OF_THIRD_TABLE_AVOIDING_DOOR=9
STATE_POUR_BALL_OUT_OF_CUP=10
STATE_STOP = 11

LEFT_HAND='l'
RIGHT_HAND='r'
BOTH_HANDS='b'

CLOSE_HAND_FINGERS=1.7
OPEN_HAND_FINGERS=0.9
PATH_TOLERANCE_VIOLATED_ERROR=-3

global velma
global planner
global current_joint_pose

current_joint_pose={'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

def initialize_velma():
    global velma
    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(3)
    # Initialize Robot
    initialize_robot()
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

# Initialize robot
def initialize_robot():
    global velma
    flag_initialize=raw_input("Press (y) if you want to initialize robot, otherwise press (n). Answer: ")
    try:
        assert(flag_initialize=='y' or flag_initialize=='n')
    except:
        return
    if flag_initialize !='y':
        return
    print "Also, head motors must be homed after start-up of the robot."
    print "Sending head pan motor START_HOMING command..."
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)
    print "Head pan motor homing successful."
    print "Sending head tilt motor START_HOMING command..."
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)
    print "Head tilt motor homing successful."

def initialize_planner():
    global planner
    print "Waiting for Planner initialization..."
    planner = Planner(velma.maxJointTrajLen())
    if not planner.waitForInit(timeout_s=10.0):
        print "Could not initialize Planner"
        exitError(2)
    print "Planner initialization ok!"

def initialize_octomap():
    global planner
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(0.3)
    octomap = oml.getOctomap(timeout_s=5.0)
    planner.processWorld(octomap)

# Switch to joint impedance mode
def switch_to_joint_impedance_mode():
    global velma
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    try:
        diag = velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            print "Error - The core_cs should be in jnt_imp state, but it is not"
            switch_to_joint_impedance_mode()
    except:
        print "Error in switch_to_joint_impedance_mode"

# Switch to cartesian impedance mode.
def switch_to_cartesian_impedance_mode(which_hand):
    try:
        assert(which_hand == LEFT_HAND or which_hand ==RIGHT_HAND or which_hand==BOTH_HANDS)
    except:
        return
    if which_hand==RIGHT_HAND or which_hand==BOTH_HANDS:
        if not velma.moveCartImpRightCurrentPos(start_time=0.2):
            exitError(10)
        if velma.waitForEffectorRight() != 0:
            print "Error - switch_to_cartesian_impedance_mode(which_hand)"
    elif which_hand==LEFT_HAND or which_hand==BOTH_HANDS:
        if not velma.moveCartImpLeftCurrentPos(start_time=0.2):
            exitError(10)
        if velma.waitForEffectorLeft() != 0:
            print "Error - switch_to_cartesian_impedance_mode(which_hand)"
    rospy.sleep(2.0) #rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"

# Rotate torso by an angle
def rotate_torso(angle):
    global current_joint_pose
    # switch to joint impedance mode
    switch_to_joint_impedance_mode()
    update_current_joint_pose()
    current_joint_pose["torso_0_joint"]=angle
    move_joint_impedance(current_joint_pose)

# Set gripper (hand and angle)
def set_grippers(which_hand, dest_q):
    global velma
    if which_hand==LEFT_HAND:
        velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandLeft() != 0:
            exitError(6)
        if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
            print "Error nr 9 with left hand: "+str(dest_q)
    elif which_hand==RIGHT_HAND:
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
            print "Error nr 9 with right hand: "+str(dest_q)

# Move robot to given position in joint state
def move_joint_impedance(joints):
    global velma
    global planner
    # switch to joint impedance mode
    switch_to_joint_impedance_mode()
    print "Planning motion to the goal position using set of all joints..."
    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(joints, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(10):
        rospy.sleep(0.2)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = planner.plan(js[1], [goal_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(joints, js[1]):
        print "Error nr 6 - Robot position in joint state is not close enough!"

# set within q_map_goal given joint value for a specific joint
def set_robot_joint(joint_name, joint_angle):
    global current_joint_pose
    current_joint_pose[joint_name]=joint_angle
    return current_joint_pose

# Move to the initial position
def move_robot_to_initial_position(if_torso, which_hand):
    global current_joint_pose
    # switch to joint impedance mode
    switch_to_joint_impedance_mode()
    # change robot joints for torso and right and left arms # get current joints for velma robot
    joint_state=velma.getLastJointState()
    current_joint_values=joint_state[1]
    if if_torso:
        set_robot_joint("torso_0_joint", 0 )
    else:
        set_robot_joint("torso_0_joint", current_joint_values["torso_0_joint"]  )
    if which_hand==RIGHT_HAND or which_hand==BOTH_HANDS:
        set_robot_joint( "right_arm_0_joint", -0.3 )
        set_robot_joint( "right_arm_1_joint", -1.8 )
        set_robot_joint( "right_arm_2_joint", 1.25 )
        set_robot_joint( "right_arm_3_joint", 0.85 )
        set_robot_joint( "right_arm_4_joint", 0 )
        set_robot_joint( "right_arm_5_joint", -0.5 )
        set_robot_joint( "right_arm_6_joint", 0 )
    else:
        set_robot_joint( "right_arm_0_joint", current_joint_values["right_arm_0_joint"] )
        set_robot_joint( "right_arm_1_joint", current_joint_values["right_arm_1_joint"] )
        set_robot_joint( "right_arm_2_joint", current_joint_values["right_arm_2_joint"] )
        set_robot_joint( "right_arm_3_joint", current_joint_values["right_arm_3_joint"] )
        set_robot_joint( "right_arm_4_joint", current_joint_values["right_arm_4_joint"] )
        set_robot_joint( "right_arm_5_joint", current_joint_values["right_arm_5_joint"] )
        set_robot_joint( "right_arm_6_joint", current_joint_values["right_arm_6_joint"] )
    if which_hand==LEFT_HAND or which_hand==BOTH_HANDS:
        set_robot_joint( "left_arm_0_joint", 0.3 )
        set_robot_joint( "left_arm_1_joint", 1.8 )
        set_robot_joint( "left_arm_2_joint", -1.25 )
        set_robot_joint( "left_arm_3_joint", -0.85 )
        set_robot_joint( "left_arm_4_joint", 0 )
        set_robot_joint( "left_arm_5_joint", 0.5 )
        set_robot_joint( "left_arm_6_joint", 0 )
    else:
        set_robot_joint( "left_arm_0_joint", current_joint_values["left_arm_0_joint"])
        set_robot_joint( "left_arm_1_joint", current_joint_values["left_arm_1_joint"] )
        set_robot_joint( "left_arm_2_joint", current_joint_values["left_arm_2_joint"] )
        set_robot_joint( "left_arm_3_joint", current_joint_values["left_arm_3_joint"] )
        set_robot_joint( "left_arm_4_joint", current_joint_values["left_arm_4_joint"] )
        set_robot_joint( "left_arm_5_joint", current_joint_values["left_arm_5_joint"] )
        set_robot_joint( "left_arm_6_joint", current_joint_values["left_arm_6_joint"] )
    # Move robot to the desired position specified by q_map_zero
    move_joint_impedance(current_joint_pose)

# go to the ready position - i.e. arms are in front of robot
def move_robot_to_ready_position(if_use_grippers, if_use_torso, which_hand):
    global current_joint_pose
    switch_to_joint_impedance_mode()
    update_current_joint_pose() # update current joint states
    if if_use_grippers:
        # close grippers
        if which_hand==BOTH_HANDS or which_hand==LEFT_HAND:
            set_grippers(LEFT_HAND, [1.57,1.57,1.57,3.14]) # for left arm
        if which_hand==BOTH_HANDS or which_hand==RIGHT_HAND:
            set_grippers(RIGHT_HAND, [1.57,1.57,1.57,3.14]) # for right arm
    # get current joints for velma robot
    joint_state=velma.getLastJointState()
    current_joint_values=joint_state[1]
    # change robot joints for right and left arm
    if if_use_torso:
        set_robot_joint( "torso_0_joint", 0 )
    else:
        set_robot_joint("torso_0_joint", current_joint_values["torso_0_joint"]  )
    if which_hand==BOTH_HANDS or which_hand==RIGHT_HAND:
        set_robot_joint( "right_arm_0_joint", -0.3 )
        set_robot_joint( "right_arm_1_joint", -1.57 )
        set_robot_joint( "right_arm_2_joint", 1.57 )
        set_robot_joint( "right_arm_3_joint", 1.7 )
        set_robot_joint( "right_arm_4_joint", 0 )
        set_robot_joint( "right_arm_5_joint", -1.57 )
        set_robot_joint( "right_arm_6_joint", 0 )
    else:
        set_robot_joint( "right_arm_0_joint", current_joint_values["right_arm_0_joint"] )
        set_robot_joint( "right_arm_1_joint", current_joint_values["right_arm_1_joint"] )
        set_robot_joint( "right_arm_2_joint", current_joint_values["right_arm_2_joint"] )
        set_robot_joint( "right_arm_3_joint", current_joint_values["right_arm_3_joint"] )
        set_robot_joint( "right_arm_4_joint", current_joint_values["right_arm_4_joint"] )
        set_robot_joint( "right_arm_5_joint", current_joint_values["right_arm_5_joint"] )
        set_robot_joint( "right_arm_6_joint", current_joint_values["right_arm_6_joint"] )
    if which_hand==BOTH_HANDS or which_hand==LEFT_HAND:
        set_robot_joint( "left_arm_0_joint", 0.3 )
        set_robot_joint( "left_arm_1_joint", 1.57 )
        set_robot_joint( "left_arm_2_joint", -1.57 )
        set_robot_joint( "left_arm_3_joint", -1.7 )
        set_robot_joint( "left_arm_4_joint", 0 )
        set_robot_joint( "left_arm_5_joint", 1.57 )
        set_robot_joint( "left_arm_6_joint", 0 )
    else:
        set_robot_joint( "left_arm_0_joint", current_joint_values["left_arm_0_joint"] )
        set_robot_joint( "left_arm_1_joint", current_joint_values["left_arm_1_joint"] )
        set_robot_joint( "left_arm_2_joint", current_joint_values["left_arm_2_joint"] )
        set_robot_joint( "left_arm_3_joint", current_joint_values["left_arm_3_joint"] )
        set_robot_joint( "left_arm_4_joint", current_joint_values["left_arm_4_joint"] )
        set_robot_joint( "left_arm_5_joint", current_joint_values["left_arm_5_joint"] )
        set_robot_joint( "left_arm_6_joint", current_joint_values["left_arm_6_joint"] )
    # move robot to specific joint positions
    move_joint_impedance(current_joint_pose)
    if if_use_grippers:
        # open grippers
        if which_hand==BOTH_HANDS or which_hand==LEFT_HAND:
            set_grippers(LEFT_HAND, [1.57,1.57,1.57,3.14]) # for left arm
        if which_hand==BOTH_HANDS or which_hand==RIGHT_HAND:
            set_grippers(RIGHT_HAND, [0,0,0,0]) # for right arm

def update_current_joint_pose():
    global current_joint_pose
    # get current joints for velma robot
    joint_state=velma.getLastJointState()
    current_joint_values=joint_state[1]
    set_robot_joint( "torso_0_joint", current_joint_values["torso_0_joint"] )
    set_robot_joint( "right_arm_0_joint", current_joint_values["right_arm_0_joint"])
    set_robot_joint( "right_arm_1_joint", current_joint_values["right_arm_1_joint"] )
    set_robot_joint( "right_arm_2_joint", current_joint_values["right_arm_2_joint"] )
    set_robot_joint( "right_arm_3_joint", current_joint_values["right_arm_3_joint"] )
    set_robot_joint( "right_arm_4_joint", current_joint_values["right_arm_4_joint"] )
    set_robot_joint( "right_arm_5_joint", current_joint_values["right_arm_5_joint"] )
    set_robot_joint( "right_arm_6_joint", current_joint_values["right_arm_6_joint"])
    set_robot_joint( "left_arm_0_joint", current_joint_values["left_arm_0_joint"] )
    set_robot_joint( "left_arm_1_joint", current_joint_values["left_arm_1_joint"] )
    set_robot_joint( "left_arm_2_joint", current_joint_values["left_arm_2_joint"] )
    set_robot_joint( "left_arm_3_joint", current_joint_values["left_arm_3_joint"] )
    set_robot_joint( "left_arm_4_joint", current_joint_values["left_arm_4_joint"] )
    set_robot_joint( "left_arm_5_joint", current_joint_values["left_arm_5_joint"] )
    set_robot_joint( "left_arm_6_joint", current_joint_values["left_arm_6_joint"] )

# Move end effector to specific position and angle, and wrench
def move_end_effector_to_cartesian_position_and_angle_and_wrench(which_hand, angleX, angleY, angleZ, x, y, z, imp_list, imp_times, max_wrench, path_tool_tolerance, time):
    global velma
    T_hand = PyKDL.Frame(PyKDL.Rotation.RPY(angleX, angleY, angleZ), PyKDL.Vector(x, y, z))
    if which_hand==RIGHT_HAND:
        if not velma.moveCartImpRight([T_hand], time, None, None, imp_list, imp_times, max_wrench, start_time=0.5, path_tol=path_tool_tolerance):
            exitError(13)
        error=velma.waitForEffectorRight()
        if error != 0:
            if error==PATH_TOLERANCE_VIOLATED_ERROR:
                return PATH_TOLERANCE_VIOLATED_ERROR
    elif which_hand==LEFT_HAND:
        if not velma.moveCartImpLeft([T_hand], [3.0], None, None, imp_list, imp_times, max_wrench, start_time=0.5, path_tol=path_tool_tolerance):
            exitError(13)
        error=velma.waitForEffectorLeft()
        if error != 0:
            if error==PATH_TOLERANCE_VIOLATED_ERROR:
                return PATH_TOLERANCE_VIOLATED_ERROR
    else:
        print "Error - none hand has been specified!"
    rospy.sleep(0.2)
    # Check precision
    if which_hand==RIGHT_HAND:
        T_hand_diff = PyKDL.diff(T_hand, velma.getTf("B", "Tr"), 1.0)
        if T_hand_diff.vel.Norm() > 0.05 or T_hand_diff.rot.Norm() > 0.05:
            print "Precision for right hand is not perfect!"
    elif which_hand==LEFT_HAND:
        print "d 1.13 - check precision for the left hand - later"
        #T_hand_diff = PyKDL.diff(T_hand, velma.getTf("B", "Tl"), 1.0)
        #if T_hand_diff.vel.Norm() > 0.05 or T_hand_diff.rot.Norm() > 0.05:
        #    print "Precision for left hand is not perfect!"

def move_in_front_of_cup(if_cup_on_table, which_hand):
    switch_to_cartesian_impedance_mode(which_hand)
    tool_name="T"
    tool_name=tool_name+which_hand
    tool_frame=velma.getTf("B", tool_name)
    tool_pose=tool_frame.p
    if if_cup_on_table:
        cup_option="cup_on_table"
    else:
        cup_option="cup_in_cabinet"
    cup_frame=velma.getTf("B", cup_option)
    cup_pose=cup_frame.p
    delta_x=0
    delta_y=0
    delta_z=0
    delta_angle=0
    if if_cup_on_table:
        delta_x=cup_pose[0]-tool_pose[0]
        delta_z=cup_pose[2]-tool_pose[2]
    else:
        delta_y=cup_pose[1]-tool_pose[1]
        delta_z=cup_pose[2]-tool_pose[2]
    move_tool_by_delta(which_hand,delta_x,delta_y, delta_z, delta_angle)
    return

def move_tool_by_delta(which_hand, d_x, d_y, d_z, d_alpha, time=DEFAULT_TIME):
    frame=velma.getTf("B","T"+which_hand)
    pose=frame.p
    angle_z=frame.M.GetRPY()[2]+d_alpha
    pose=PyKDL.Vector(pose[0]+d_x, pose[1]+d_y, pose[2]+d_z)
    move_end_effector_to_cartesian_position_and_angle_and_wrench(which_hand, 0, 0, angle_z, pose[0], pose[1], pose[2], IMP_LIST_DEFAULT_STIFFNESS, IMP_TIME_LIST, MAX_WRENCH_DEFAULT, PATH_TOLERANCE_DEFAULT_2, DEFAULT_TIME)

def move_and_rotate_tool_by_delta(which_hand, d_x, d_y, d_z, d_alpha_x, d_alpha_y, d_alpha_z, time=DEFAULT_TIME):
    try:
        frame=velma.getTf("B","T"+which_hand)
        pose=frame.p
        angle_x=frame.M.GetRPY()[0]+d_alpha_x
        angle_y=frame.M.GetRPY()[1]+d_alpha_y
        angle_z=frame.M.GetRPY()[2]+d_alpha_z
        pose=PyKDL.Vector(pose[0]+d_x, pose[1]+d_y, pose[2]+d_z)
        move_end_effector_to_cartesian_position_and_angle_and_wrench(which_hand, angle_x, angle_y, angle_z, pose[0], pose[1], pose[2], IMP_LIST_DEFAULT_STIFFNESS, IMP_TIME_LIST, MAX_WRENCH_DEFAULT, PATH_TOLERANCE_DEFAULT_2, time)
    except:
        print "Error - move_tool_by_delta"

def check_object_frame(specific_frame, object_name):
    try:
        T_B_object = velma.getTf(specific_frame, object_name)
        return T_B_object
    except:
        print "Error - check_object_frame - tf: "+str(object_name)+" does not exist!"


######################## -- END -- Auxiliary subsystem script ########################


