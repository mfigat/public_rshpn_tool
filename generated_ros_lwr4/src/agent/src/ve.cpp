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

#ifndef VE_H
#define VE_H

#include "auxiliary_functions.h"
#include "auxiliary_agent.h"
#include "auxiliary_ve.h"

namespace agent{
	class ve{
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

		std::vector<ros::Publisher > _vectorOfSenderDiagnostics;
		ros::Subscriber _subscriberdesired_x;
		ros::Subscriber _subscriberdesired_y;
		ros::Subscriber _subscriberdesired_z;

		/* Buffers */
		/* Input Buffers */
		std_msgs::Float64 desired_x;
		std_msgs::Float64 desired_y;
		std_msgs::Float64 desired_z;

		/* Output Buffers */

		/* Internal Memory Buffers */
	public:
		ve();
		/* updates current state based on initial condition */
		void updateCurrentState();
		void startSubsystem();
		void updatedesired_x(const std_msgs::Float64::ConstPtr& msg);
		void updatedesired_y(const std_msgs::Float64::ConstPtr& msg);
		void updatedesired_z(const std_msgs::Float64::ConstPtr& msg);
		/* Return data from output buffers */

		/* Return data from input buffers */
		std_msgs::Float64 getdesired_x();
		std_msgs::Float64 getdesired_y();
		std_msgs::Float64 getdesired_z();

		/* Return data from internal buffers */

		/* Set Input Buffers */
		void setdesired_x(std_msgs::Float64 data);
		void setdesired_y(std_msgs::Float64 data);
		void setdesired_z(std_msgs::Float64 data);

		/* Set Internal Buffers */

		/* Set Output Buffers */

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

		/* Behaviour Init */
		/* Terminal Condition */
		bool terminalCondition_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Error Condition */
		bool errorCondition_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Transition function */
		void transitionFunction_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Send data to other subsystems */
		void sendData_Init(/*  */);
		/* Receive data from other subsystems */
		void receiveData_Init(/* */);
		/* Execute behaviour Init*/
		void executeBehaviour_Init(/* */);

		/* Behaviour Move */
		/* Terminal Condition */
		bool terminalCondition_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Error Condition */
		bool errorCondition_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Transition function */
		void transitionFunction_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */);
		/* Send data to other subsystems */
		void sendData_Move(/*  */);
		/* Receive data from other subsystems */
		void receiveData_Move(/* */);
		/* Execute behaviour Move*/
		void executeBehaviour_Move(/* */);

		/* Declaration of functions responsible for switching subsystem ve between states : S_Init S_Move */
		/* State S_Init: */
		void subsystemState_S_Init();
		/* State S_Move: */
		void subsystemState_S_Move();
		/* Initial conditions */
		/* Initial condition for state S_Init: switching to state S_Move*/
		bool initialCondition_From_S_Init_To_S_Move();
	}; // END OF ve
} // END OF agent


agent::ve::ve(){
	_n = new ros::NodeHandle();
	_subsystemName="ve";
	_subsystemFrequency=200;
	_currentSubsystemState="S_Init";
	initialiseModel();
}

/* Start subsystem */
void agent::ve::startSubsystem(){
	try{
		do{
			/* Execute behaviour associated with _currentSubsystemState -- choose appropriate state based on _currentSubsystemState */
			if(_currentSubsystemState=="S_Init"){
				subsystemState_S_Init();
				continue;
			}
			if(_currentSubsystemState=="S_Move"){
				subsystemState_S_Move();
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

void agent::ve::updatedesired_x(const std_msgs::Float64::ConstPtr& msg){
	desired_x=*msg;
}

void agent::ve::updatedesired_y(const std_msgs::Float64::ConstPtr& msg){
	desired_y=*msg;
}

void agent::ve::updatedesired_z(const std_msgs::Float64::ConstPtr& msg){
	desired_z=*msg;
}

void agent::ve::setdesired_x(std_msgs::Float64 data){
	desired_x=data;
}

void agent::ve::setdesired_y(std_msgs::Float64 data){
	desired_y=data;
}

void agent::ve::setdesired_z(std_msgs::Float64 data){
	desired_z=data;
}

std::string agent::ve::getSubsystemName(){
	return _subsystemName;
}

void agent::ve::printInfo(std::string str){
	std::cout<<"["<<_subsystemName<<"] -- info: "<<str<<std::endl;
}

void agent::ve::printError(std::string str){
	std::cerr<<"["<<_subsystemName<<"] -- error : "<<str<<std::endl;
}

void agent::ve::printLog(std::string str){
	#if PRINT_LOG
		std::clog<<"["<<_subsystemName<<"] -- log : "<<str<<std::endl;
	#endif
}

double agent::ve::get_subsystemFrequency(){
	return _subsystemFrequency;
}

void agent::ve::initialiseModel(){
	initialiseSendChannel();
	initialiseSendChannelForDiagnostics();
	initialiseReceiveChannel();
}

void agent::ve::initialiseSendChannel(){
}

void agent::ve::initialiseSendChannelForDiagnostics(){
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("ve/_currentSubsystemState", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("ve/_subsystemFrequency", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("ve/_subsystemName", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_subsystemIterations", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_behaviourIterations", CHANNEL_SIZE));
}

void agent::ve::initialiseReceiveChannel(){
	_subscriberdesired_x=_n->subscribe("desired_x", CHANNEL_SIZE, &agent::ve::updatedesired_x, this);
	_subscriberdesired_y=_n->subscribe("desired_y", CHANNEL_SIZE, &agent::ve::updatedesired_y, this);
	_subscriberdesired_z=_n->subscribe("desired_z", CHANNEL_SIZE, &agent::ve::updatedesired_z, this);
}

/* Return data from output buffers */
/* Return data from input buffers */
std_msgs::Float64 agent::ve::getdesired_x(){	return desired_x;	}

std_msgs::Float64 agent::ve::getdesired_y(){	return desired_y;	}

std_msgs::Float64 agent::ve::getdesired_z(){	return desired_z;	}

/* Return data from internal buffers */
/* Send (publish) diagnostic data */
void agent::ve::sendDataForDiagnostics(/* subsystem diagnostic state */){
	std_msgs::String temporarString;
	std_msgs::Int64 temporarInt;
	std_msgs::Float64 temporarFloat;
	temporarString.data=_currentSubsystemState;
	_vectorOfSenderDiagnostics[0].publish(temporarString);
	temporarFloat.data=_subsystemFrequency;
	_vectorOfSenderDiagnostics[1].publish(temporarFloat);
	temporarString.data=_subsystemName;
	_vectorOfSenderDiagnostics[2].publish(temporarString);
	temporarInt.data=_subsystemIterations;
	_vectorOfSenderDiagnostics[3].publish(temporarInt);
	temporarInt.data=_behaviourIterations;
	_vectorOfSenderDiagnostics[4].publish(temporarInt);
}
/* Behaviour definitions */
/* Behaviour Init */
/* Terminal Condition */
bool agent::ve::terminalCondition_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	return  true ;
}
/* Error Condition */
bool agent::ve::errorCondition_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_Init(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	/* Partial transition function - name - tf1_1*/
			
}
/* Send data to other subsystems */
void agent::ve::sendData_Init(/*  */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Init] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_Init(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Init] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Init*/
void agent::ve::executeBehaviour_Init(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Init */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Init();
		/* Sends data! */
		sendData_Init();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Init();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Init() || errorCondition_Init();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour Move */
/* Terminal Condition */
bool agent::ve::terminalCondition_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	return  false ;
}
/* Error Condition */
bool agent::ve::errorCondition_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_Move(/* std_msgs::Float64desired_x,std_msgs::Float64desired_y,std_msgs::Float64desired_z */){
	/* Partial transition function - name - tf1_1*/
	x_d=desired_x.data;
	y_d=desired_y.data;
	z_d=desired_z.data;
	for(int i=0;i<7;i++){
		torque_grav[i]=0;
		torque_impedance[i]=0;
	}		
	// read data from shared memory
	tmp=sm_consumer.readAsynchronously();
	// ############## Gravitation compensation ################# */
	lwr.calculateGravitationCompensationTorque(tmp.theta1, tmp.theta2, tmp.theta3, tmp.theta4, tmp.theta5, tmp.theta6, tmp.theta7);
	// ############### Impedance control ############ */
	torque_impedance=lwr.impedanceControlInCartesianSpace(tmp.x_current, tmp.y_current, tmp.z_current, tmp.v_x, tmp.v_y, tmp.v_z, x_d, y_d, z_d);
	/* ################ Add two torques */
	for(int i=0;i<7;i++){
		msg_tmp._joints[i] = torque_impedance[i] + torque_grav[i];
	  // std::cout <<"[Torque - calculated] torque"<<i+1<<"="<<msg_tmp._joints[i]<<std::endl;
	}
	/* ######## Send calculated torque ######## */
	sm_producer.writeAsynchronously(msg_tmp);
								
}
/* Send data to other subsystems */
void agent::ve::sendData_Move(/*  */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Move] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_Move(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Move] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Move*/
void agent::ve::executeBehaviour_Move(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Move */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Move();
		/* Sends data! */
		sendData_Move();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Move();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Move() || errorCondition_Move();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}

/* Definition of functions responsible for switching subsystem ve between states : S_Init S_Move */
/* State S_Init: */
void agent::ve::subsystemState_S_Init(){
	/* Executing behaviour Init */
	executeBehaviour_Init(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Init: switching to state S_Move*/
	if(initialCondition_From_S_Init_To_S_Move()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_Move";
	}
}

/* State S_Move: */
void agent::ve::subsystemState_S_Move(){
	/* Executing behaviour Move */
	executeBehaviour_Move(/* */);
	/* Behaviour has been terminated */
}

/* Initial condition for state S_Init: switching to state S_Move*/
bool agent::ve::initialCondition_From_S_Init_To_S_Move(){
	/* Initial condition specified by user */
	return  true ;
}

/* ############################################### */
/* ############### MAIN FUNCTION ################# */
/* ############################################### */
int main(int argc, char ** argv){
	ros::init(argc, argv, "ve");
	/* Initialize subsystem */
	agent::ve *s=new agent::ve();

	/* Start subsystem: ve */
	s->startSubsystem();

	return 0;

} /* END OF MAIN FUNCTION */
#endif // VE
