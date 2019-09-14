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

#ifndef AUXILIARY_FUNCTIONS_H
#define AUXILIARY_FUNCTIONS_H

#include "ros/ros.h"
#include <ros/duration.h>

#define DEFAULT_ROS_FREQUENCY 1

class AuxiliaryFunctions{
	private:
		ros::Rate * _loop_rate;
	public:
		AuxiliaryFunctions();
		AuxiliaryFunctions(double frequency);
		/* check whether the ros is ok and there were no SIGINT signal sent */
		static bool isSubsystemOK();
		/* shutdown subsystem */
		static void shutdownSubsystem();
		/* set subsystem frequency */
		static ros::Rate getLoopRate(double frequency);
		/* sleep */
		void sleep();
};

AuxiliaryFunctions::AuxiliaryFunctions(){ _loop_rate=new ros::Rate(DEFAULT_ROS_FREQUENCY); }

AuxiliaryFunctions::AuxiliaryFunctions(double frequency){ _loop_rate=new ros::Rate(frequency); }

bool AuxiliaryFunctions::isSubsystemOK(){ return ros::ok(); }

void AuxiliaryFunctions::shutdownSubsystem(){ ros::shutdown(); }

void AuxiliaryFunctions::sleep(){ _loop_rate->sleep(); }

ros::Rate AuxiliaryFunctions::getLoopRate(double frequency){ return ros::Rate(frequency); }


#endif /* END OF HEADER */

