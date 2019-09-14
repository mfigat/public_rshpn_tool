/* FILE GENERATED AUTOMATICALY */
/*
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

*/

#ifndef CS_H
#define CS_H

#include "auxiliary_functions.h"
#include "auxiliary_agent.h"
#include "auxiliary_cs.h"

namespace agent{
	class cs{
	private:
		/* ROS node */
		ros::NodeHandle * _n;

		/* Current subsystem state name */
		std::string _currentSubsystemState;

		double _subsystemFrequency;
		std::string _subsystemName;

		/* Subsystem iterations */
		int _subsystemIterations;

		/* Behaviour iterations */
		int _behaviourIterations;

		ros::Publisher _senderdesired_x;
		ros::Publisher _senderdesired_y;
		ros::Publisher _senderdesired_z;
		std::vector<ros::Publisher > _vectorOfSenderDiagnostics;

		/* Buffers */
		/* Input Buffers */

		/* Output Buffers */
		std_msgs::Float64 desired_x;
		std_msgs::Float64 desired_y;
		std_msgs::Float64 desired_z;

		/* Internal Memory Buffers */
		std_msgs::Float64 flag;
	public:
		cs();
		/* updates current state based on initial condition */
		void updateCurrentState();
		void startSubsystem();
		/* Return data from output buffers */
		std_msgs::Float64 getdesired_x();
		std_msgs::Float64 getdesired_y();
		std_msgs::Float64 getdesired_z();

		/* Return data from input buffers */

		/* Return data from internal buffers */
		std_msgs::Float64 getflag();

		/* Set Input Buffers */

		/* Set Internal Buffers */
		void setflag(std_msgs::Float64 data);

		/* Set Output Buffers */
		void setdesired_x(std_msgs::Float64 data);
		void setdesired_y(std_msgs::Float64 data);
		void setdesired_z(std_msgs::Float64 data);

		std::string getSubsystemName();
		void printInfo(std::string str);
		void printError(std::string str);
		void printLog(std::string str);
		double get_subsystemFrequency();
		/* Initialise communication model */
		void initialiseModel();
		void initialiseSendChannel();
		void initialiseSendChannelForDiagnostics();
		void initialiseReceiveChannel();
		void sendDataForDiagnostics();
		/* Behaviour declarations */

		/* Behaviour Idle */
		/* Terminal Condition */
		bool terminalCondition_Idle(/* std_msgs::Float64flag */);
		/* Error Condition */
		bool errorCondition_Idle(/* std_msgs::Float64flag */);
		/* Transition function */
		void transitionFunction_Idle(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z,std_msgs::Float64flag */);
		/* Send data to other subsystems */
		void sendData_Idle(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Receive data from other subsystems */
		void receiveData_Idle(/* */);
		/* Execute behaviour Idle*/
		void executeBehaviour_Idle(/* */);

		/* Behaviour MoveTo */
		/* Terminal Condition */
		bool terminalCondition_MoveTo(/* std_msgs::Float64flag */);
		/* Error Condition */
		bool errorCondition_MoveTo(/* std_msgs::Float64flag */);
		/* Transition function */
		void transitionFunction_MoveTo(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z,std_msgs::Float64flag */);
		/* Send data to other subsystems */
		void sendData_MoveTo(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Receive data from other subsystems */
		void receiveData_MoveTo(/* */);
		/* Execute behaviour MoveTo*/
		void executeBehaviour_MoveTo(/* */);

		/* Declaration of functions responsible for switching subsystem cs between states : S_Idle S_MoveTo */
		/* State S_Idle: */
		void subsystemState_S_Idle();
		/* State S_MoveTo: */
		void subsystemState_S_MoveTo();
		/* Initial conditions */
		/* Initial condition for state S_Idle: switching to state S_MoveTo*/
		bool initialCondition_From_S_Idle_To_S_MoveTo();
	}; // END OF cs
} // END OF agent


agent::cs::cs(){
	_n = new ros::NodeHandle();
	_subsystemName="cs";
	_subsystemFrequency=200;
	_currentSubsystemState="S_Idle";
	initialiseModel();
}

/* Start subsystem */
void agent::cs::startSubsystem(){
	try{
		do{
			/* Execute behaviour associated with _currentSubsystemState -- choose appropriate state based on _currentSubsystemState */
			if(_currentSubsystemState=="S_Idle"){
				subsystemState_S_Idle();
				continue;
			}
			if(_currentSubsystemState=="S_MoveTo"){
				subsystemState_S_MoveTo();
				continue;
			}
		}
		while(AuxiliaryFunctions::isSubsystemOK());
	}
	catch(...){
		printError("Current state is out of the bound -- method startSubsystem -- file subsystem.h!");
		AuxiliaryFunctions::shutdownSubsystem();
	}
}

void agent::cs::setflag(std_msgs::Float64 data){
	flag=data;
}

void agent::cs::setdesired_x(std_msgs::Float64 data){
	desired_x=data;
}

void agent::cs::setdesired_y(std_msgs::Float64 data){
	desired_y=data;
}

void agent::cs::setdesired_z(std_msgs::Float64 data){
	desired_z=data;
}

std::string agent::cs::getSubsystemName(){
	return _subsystemName;
}

void agent::cs::printInfo(std::string str){
	std::cout<<"["<<_subsystemName<<"] -- info: "<<str<<std::endl;
}

void agent::cs::printError(std::string str){
	std::cerr<<"["<<_subsystemName<<"] -- error : "<<str<<std::endl;
}

void agent::cs::printLog(std::string str){
	#if PRINT_LOG
		std::clog<<"["<<_subsystemName<<"] -- log : "<<str<<std::endl;
	#endif
}

double agent::cs::get_subsystemFrequency(){
	return _subsystemFrequency;
}

void agent::cs::initialiseModel(){
	initialiseSendChannel();
	initialiseSendChannelForDiagnostics();
	initialiseReceiveChannel();
}

void agent::cs::initialiseSendChannel(){
		_senderdesired_x=_n->advertise<std_msgs::Float64>("desired_x", CHANNEL_SIZE);
		_senderdesired_y=_n->advertise<std_msgs::Float64>("desired_y", CHANNEL_SIZE);
		_senderdesired_z=_n->advertise<std_msgs::Float64>("desired_z", CHANNEL_SIZE);
}

void agent::cs::initialiseSendChannelForDiagnostics(){
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("cs/flag", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("cs/_currentSubsystemState", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("cs/_subsystemFrequency", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("cs/_subsystemName", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("cs/_subsystemIterations", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("cs/_behaviourIterations", CHANNEL_SIZE));
}

void agent::cs::initialiseReceiveChannel(){
}

/* Return data from output buffers */
std_msgs::Float64 agent::cs::getdesired_x(){	return desired_x;	}

std_msgs::Float64 agent::cs::getdesired_y(){	return desired_y;	}

std_msgs::Float64 agent::cs::getdesired_z(){	return desired_z;	}

/* Return data from input buffers */
/* Return data from internal buffers */
std_msgs::Float64 agent::cs::getflag(){	return flag;	}

/* Send (publish) diagnostic data */
void agent::cs::sendDataForDiagnostics(/* subsystem diagnostic state */){
	_vectorOfSenderDiagnostics[0].publish(flag);	std_msgs::String temporarString;
	std_msgs::Int64 temporarInt;
	std_msgs::Float64 temporarFloat;
	temporarString.data=_currentSubsystemState;
	_vectorOfSenderDiagnostics[1].publish(temporarString);
	temporarFloat.data=_subsystemFrequency;
	_vectorOfSenderDiagnostics[2].publish(temporarFloat);
	temporarString.data=_subsystemName;
	_vectorOfSenderDiagnostics[3].publish(temporarString);
	temporarInt.data=_subsystemIterations;
	_vectorOfSenderDiagnostics[4].publish(temporarInt);
	temporarInt.data=_behaviourIterations;
	_vectorOfSenderDiagnostics[5].publish(temporarInt);
}
/* Behaviour definitions */
/* Behaviour Idle */
/* Terminal Condition */
bool agent::cs::terminalCondition_Idle(/* std_msgs::Float64flag */){
	return ( true );
}
/* Error Condition */
bool agent::cs::errorCondition_Idle(/* std_msgs::Float64flag */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_Idle(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z,std_msgs::Float64flag */){
	 
	flag.data=1;
}
/* Send data to other subsystems */
void agent::cs::sendData_Idle(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_senderdesired_x.publish(desired_x);
	_senderdesired_y.publish(desired_y);
	_senderdesired_z.publish(desired_z);
	/* END OF TYPICAL SEND CALL */

	/* 	Buffer name: desired_x
		 Utilised to send data of type: std_msgs::Float64
		On communication channel: desired_x
		Define activity for a such buffer! */
	/* 	Buffer name: desired_y
		 Utilised to send data of type: std_msgs::Float64
		On communication channel: desired_y
		Define activity for a such buffer! */
	/* 	Buffer name: desired_z
		 Utilised to send data of type: std_msgs::Float64
		On communication channel: desired_z
		Define activity for a such buffer! */
}
/* Receive data from other subsystems */
void agent::cs::receiveData_Idle(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

}
/* Execute behaviour Idle*/
void agent::cs::executeBehaviour_Idle(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Idle */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Idle();
		/* Sends data! */
		sendData_Idle();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Idle();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Idle() || errorCondition_Idle();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour MoveTo */
/* Terminal Condition */
bool agent::cs::terminalCondition_MoveTo(/* std_msgs::Float64flag */){
	return   false ;
}
/* Error Condition */
bool agent::cs::errorCondition_MoveTo(/* std_msgs::Float64flag */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_MoveTo(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z,std_msgs::Float64flag */){
	/* Partial transition function - name - tf1_1*/
	// calculate next desired x,y,z coordinates
	LWR4KinematicsDynamics::calculateNextEndEffectorPosition_Circle_ZY_Plane(_iterations2, _iterations, x_d, y_d, z_d, _flag, 0.3, 0, 0, 0.5);
	desired_x.data=x_d;
	desired_y.data=y_d;
	desired_z.data=z_d;
										
}
/* Send data to other subsystems */
void agent::cs::sendData_MoveTo(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_senderdesired_x.publish(desired_x);
	_senderdesired_y.publish(desired_y);
	_senderdesired_z.publish(desired_z);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour MoveTo] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_MoveTo(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour MoveTo] -- Receiving Data"<<std::endl;
}
/* Execute behaviour MoveTo*/
void agent::cs::executeBehaviour_MoveTo(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour MoveTo */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_MoveTo();
		/* Sends data! */
		sendData_MoveTo();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_MoveTo();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_MoveTo() || errorCondition_MoveTo();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}

/* Definition of functions responsible for switching subsystem cs between states : S_Idle S_MoveTo */
/* State S_Idle: */
void agent::cs::subsystemState_S_Idle(){
	/* Executing behaviour Idle */
	executeBehaviour_Idle(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Idle: switching to state S_MoveTo*/
	if(initialCondition_From_S_Idle_To_S_MoveTo()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_MoveTo";
	}
}

/* State S_MoveTo: */
void agent::cs::subsystemState_S_MoveTo(){
	/* Executing behaviour MoveTo */
	executeBehaviour_MoveTo(/* */);
	/* Behaviour has been terminated */
}

/* Initial condition for state S_Idle: switching to state S_MoveTo*/
bool agent::cs::initialCondition_From_S_Idle_To_S_MoveTo(){
	/* Initial condition specified by user */
	return  true ;
}

/* ############################################### */
/* ############### MAIN FUNCTION ################# */
/* ############################################### */
int main(int argc, char ** argv){
	ros::init(argc, argv, "cs");
	/* Initialize subsystem */
	agent::cs *s=new agent::cs();

	/* Start subsystem: cs */
	s->startSubsystem();

	return 0;

} /* END OF MAIN FUNCTION */
#endif // CS
