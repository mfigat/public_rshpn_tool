#include "lwr4_kinematics_dynamics.h"

/* Constructor with parameters determining stiffness and damping matrixes for impedance control in cartesian space */
LWR4KinematicsDynamics::LWR4KinematicsDynamics(double _k, double _k_diag, double _d, double _d_diag){ //: k(_k), k_diag(k_diag), d(_d), d_diag(_d_diag){ -- does not work properly

	/* SET manipulator masses */
	m1=MASS_1;
	m2=MASS_2;
	m3=MASS_3;
	m4=MASS_4;
	m5=MASS_5;
	m6=MASS_6;
	m7=MASS_7;
	/* END MASSES */

	/* Denvaite Hartenberg parameter of LWR4+ manipulator, i.e. d parameter */
	d1=D1;
	d3=D3;
	d5=D5;
	d7=D7;
	/* END D-H parameter */

	/* Gravitation */
	g=gravity_constant;
	/* END Gravitation */


	/* ERROR - does not work properly setting class parameters */
	k=_k;
	k_diag=_k_diag;
	d=_d;
	d_diag=_d_diag;


	std::cout<<"k_diag="<<k_diag<<" _k_diag="<<_k_diag<<std::endl;
	std::cout<<"d_diag="<<d_diag<<" _d_diag="<<_d_diag<<std::endl;

} /* END LWR4KinematicsDynamics */


int LWR4KinematicsDynamics::function(int i){
	#ifdef DEBUG_SHARED_MEMORY
		std::cout<<"Library function "<<i<<std::endl;
	#endif
	std::cout<<"Test LWR4KinematicsDynamics function"<<std::endl;
	return 2*i;

}



std::array<double,7> LWR4KinematicsDynamics::calculateGravitationCompensationTorque(double _theta1, double _theta2, double _theta3, double _theta4, double _theta5, double _theta6, double _theta7) {
	//
	#ifdef DEBUG_SHARED_MEMORY
		std::cout<<"Gravitation compensation based on Newton-Euler dynamics equations"<<std::endl;
	#endif

	/* UPDATE theta_i, where i={0,...6} */
	theta1=_theta1;
	theta2=_theta2;
	theta3=_theta3;
	theta4=_theta4;
	theta5=_theta5;
	theta6=_theta6;
	theta7=_theta7;
	/* END UPDATE theta */

	/* UPDATE SIN and COS */
	// cosinus
	c1=cos(theta1); c2=cos(theta2); c3=cos(theta3); c4=cos(theta4); c5=cos(theta5); c6=cos(theta6); c7=cos(theta7);
	// sinus
	s1=sin(theta1); s2=sin(theta2); s3=sin(theta3); s4=sin(theta4); s5=sin(theta5); s6=sin(theta6); s7=sin(theta7);

	/* END - SIN and COS*/

	/* Calculate torques counterbalancing the gravitation */
	torque[6]=calculate_gravity_compensation_joint_7();
	torque[5]=calculate_gravity_compensation_joint_6();
	torque[4]=calculate_gravity_compensation_joint_5();
	torque[3]=calculate_gravity_compensation_joint_4();
	torque[2]=calculate_gravity_compensation_joint_3();
	torque[1]=calculate_gravity_compensation_joint_2();
	torque[0]=calculate_gravity_compensation_joint_1();
	/* END */

	#ifdef DEBUG_SHARED_MEMORY
		for (int i = 0; i < 7; ++i) {
				std::cout<<"[Gravitation compensation] torque["<<i<<"]="<<torque[i]<<std::endl;
		}
	#endif

	return torque;
} /* END calculateGravitationCompensationTorque */


double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_1(){
	double tau=-g*(c3*c4*c4*d3*m4*s2*s2*s3 - c3*d3*m4*s2*s2*s3 - c3*c5*c5*d3*m5*s2*s2*s3 - c3*c5*c5*d3*m6*s2*s2*s3 - c2*c2*c5*d5*m6*s4*s4*s5 + c3*d3*m4*s2*s2*s3*s4*s4 + c3*d3*m5*s2*s2*s3*s4*s4 - c3*d3*m5*s2*s2*s3*s5*s5 + c5*d5*m6*s2*s2*s3*s3*s5 + c3*c4*d5*m6*s2*s2*s3*s5*s5 + c4*c5*d3*m6*s2*s2*s3*s3*s5 + c2*c4*d3*m5*s2*s3*s4 + c3*c4*c4*c5*c5*d3*m5*s2*s2*s3 - c3*c3*c4*c4*c5*d5*m6*s2*s2*s5 + c2*c2*c5*c6*c6*d5*m6*s4*s4*s5 + c3*c4*c4*d3*m5*s2*s2*s3*s5*s5 + c3*c4*c4*d3*m6*s2*s2*s3*s5*s5 + c3*c6*c6*d3*m6*s2*s2*s3*s4*s4 - c3*c6*c6*d3*m6*s2*s2*s3*s5*s5 - c5*c6*c6*d5*m6*s2*s2*s3*s3*s5 + c2*c2*c5*d5*m6*s4*s4*s5*s6*s6 + c3*d3*m6*s2*s2*s3*s4*s4*s6*s6 - c3*d3*m6*s2*s2*s3*s5*s5*s6*s6 - c5*d5*m6*s2*s2*s3*s3*s5*s6*s6 + c2*c5*c5*d5*m6*s2*s3*s4 - c2*d5*m6*s2*s3*s4*s5*s5 - c3*c4*c5*c5*d5*m6*s2*s2*s3 - c3*c3*c4*c5*d3*m6*s2*s2*s5 - c2*c4*c5*c5*d3*m5*s2*s3*s4 + c2*c4*c6*c6*d3*m6*s2*s3*s4 + c3*c4*c4*c5*c5*c6*c6*d3*m6*s2*s2*s3 + c3*c3*c4*c4*c5*c6*c6*d5*m6*s2*s2*s5 - c2*c4*d3*m5*s2*s3*s4*s5*s5 - c2*c4*d3*m6*s2*s3*s4*s5*s5 + c2*c4*d3*m6*s2*s3*s4*s6*s6 + c3*c4*c4*c5*c5*d3*m6*s2*s2*s3*s6*s6 + c3*c3*c4*c4*c5*d5*m6*s2*s2*s5*s6*s6 - c2*c5*c5*c6*c6*d5*m6*s2*s3*s4 - c2*c5*c5*d5*m6*s2*s3*s4*s6*s6 + c2*c6*c6*d5*m6*s2*s3*s4*s5*s5 + c2*d5*m6*s2*s3*s4*s5*s5*s6*s6 + c2*c3*c5*d3*m6*s2*s4*s5 + c3*c4*c5*c5*c6*c6*d5*m6*s2*s2*s3 + c3*c3*c4*c5*c6*c6*d3*m6*s2*s2*s5 + c3*c4*c5*c5*d5*m6*s2*s2*s3*s6*s6 - c3*c4*c6*c6*d5*m6*s2*s2*s3*s5*s5 - c4*c5*c6*c6*d3*m6*s2*s2*s3*s3*s5 + c3*c3*c4*c5*d3*m6*s2*s2*s5*s6*s6 - c3*c4*d5*m6*s2*s2*s3*s5*s5*s6*s6 - c4*c5*d3*m6*s2*s2*s3*s3*s5*s6*s6 + 2*c2*c3*c4*c5*d5*m6*s2*s4*s5 - c2*c3*c5*c6*c6*d3*m6*s2*s4*s5 - c2*c3*c5*d3*m6*s2*s4*s5*s6*s6 - c2*c4*c5*c5*c6*c6*d3*m6*s2*s3*s4 - c2*c4*c5*c5*d3*m6*s2*s3*s4*s6*s6 - 2*c2*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2*c2*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6);
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_2(){
	double tau=-(g*(140*m2*s2 - 140*c3*c3*m3*s2 - 140*m3*s2*s3*s3 - 120*c2*m3*s3 - 160*c2*c3*m4*s4 + 120*c2*c4*c4*m4*s3 + 160*c3*c3*c4*m4*s2 + 2000*c3*c3*d3*m3*s2 + 120*c2*m4*s3*s4*s4 + 160*c4*m4*s2*s3*s3 + 2000*d3*m3*s2*s3*s3 + 2000*d3*m4*s2*s3*s3 - 152*c3*c3*c4*c5*c5*m5*s2 + 2000*c3*c3*c4*c4*d3*m4*s2 - 152*c4*c5*c5*m5*s2*s3*s3 - 152*c3*c3*c4*m5*s2*s5*s5 + 2000*c3*c3*d3*m4*s2*s4*s4 + 2000*c3*c3*d3*m5*s2*s4*s4 + 2000*c5*c5*d3*m5*s2*s3*s3 + 2000*c5*c5*d3*m6*s2*s3*s3 - 152*c4*m5*s2*s3*s3*s5*s5 + 2000*d3*m5*s2*s3*s3*s5*s5 + 152*c2*c3*c5*c5*m5*s4 + 152*c2*c3*m5*s4*s5*s5 + 2000*c3*c3*d3*m6*s2*s4*s4*s6*s6 + 2000*c6*c6*d3*m6*s2*s3*s3*s5*s5 + 2000*d3*m6*s2*s3*s3*s5*s5*s6*s6 - 125*c2*c3*c5*c5*c6*m6*s4 - 2000*c2*c3*c5*c5*d5*m5*s4 - 125*c2*c3*c6*m6*s4*s5*s5 - 2000*c2*c3*d5*m5*s4*s5*s5 - 2000*c2*c3*d5*m6*s4*s5*s5 - 125*c2*c4*c4*m6*s3*s5*s6 + 125*c3*c3*c5*m6*s2*s4*s6 - 125*c2*m6*s3*s4*s4*s5*s6 + 125*c5*m6*s2*s3*s3*s4*s6 + 125*c3*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c3*c4*c5*c5*d5*m5*s2 + 125*c4*c5*c5*c6*m6*s2*s3*s3 + 125*c3*c3*c4*c6*m6*s2*s5*s5 + 2000*c4*c5*c5*d5*m5*s2*s3*s3 + 2000*c3*c3*c4*d5*m5*s2*s5*s5 + 2000*c4*c5*c5*d5*m6*s2*s3*s3 + 2000*c3*c3*c4*d5*m6*s2*s5*s5 + 125*c4*c6*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m5*s2*s3*s3*s5*s5 + 125*c2*c3*c4*c5*m6*s6 + 2000*c2*c3*c4*d3*m5*s4 + 2000*c3*c3*c4*c4*c5*c5*d3*m5*s2 + 2000*c3*c3*c4*c4*d3*m5*s2*s5*s5 + 2000*c3*c3*c4*c4*d3*m6*s2*s5*s5 + 2000*c3*c3*c6*c6*d3*m6*s2*s4*s4 + 2000*c3*c3*c4*c5*c5*c6*c6*d5*m6*s2 - 2000*c2*c5*d3*m6*s3*s4*s5 + 2000*c3*c5*d5*m6*s2*s3*s5 + 2000*c3*c3*c4*c5*c5*d5*m6*s2*s6*s6 + 2000*c4*c6*c6*d5*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m6*s2*s3*s3*s5*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*d3*m5*s4 + 2000*c2*c3*c4*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*d3*m5*s4*s5*s5 - 2000*c2*c3*c4*d3*m6*s4*s5*s5 + 2000*c2*c3*c4*d3*m6*s4*s6*s6 + 2000*c3*c3*c4*c4*c5*c5*c6*c6*d3*m6*s2 + 2000*c3*c3*c4*c4*c5*c5*d3*m6*s2*s6*s6 - 2000*c2*c3*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c3*c5*c5*d5*m6*s4*s6*s6 + 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5 + 2000*c2*c5*c6*c6*d3*m6*s3*s4*s5 - 2000*c3*c5*c6*c6*d5*m6*s2*s3*s5 + 2000*c2*c5*d3*m6*s3*s4*s5*s6*s6 - 2000*c3*c5*d5*m6*s2*s3*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*c5*c5*d3*m6*s4*s6*s6 + 4000*c3*c4*c5*d3*m6*s2*s3*s5 - 2000*c2*c4*c5*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*c6*c6*d3*m6*s2*s3*s5 + 2000*c2*c4*c5*c6*c6*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*d3*m6*s2*s3*s5*s6*s6 + 2000*c2*c4*c5*d5*m6*s3*s4*s5*s6*s6 - 2000*c3*c4*c4*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5*s6*s6))/2000.0;
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_3(){
	double tau=-(g*(160*m4*s2*s3*s4 - 120*c3*m3*s2 + 120*c3*c4*c4*m4*s2 + 120*c3*m4*s2*s4*s4 - 152*c5*c5*m5*s2*s3*s4 - 152*m5*s2*s3*s4*s5*s5 - 2000*c2*c5*d5*m6*s4*s4*s5 - 125*c3*c4*c4*m6*s2*s5*s6 + 125*c5*c5*c6*m6*s2*s3*s4 + 2000*c5*c5*d5*m5*s2*s3*s4 + 2000*c5*c5*d5*m6*s2*s3*s4 - 125*c3*m6*s2*s4*s4*s5*s6 + 125*c6*m6*s2*s3*s4*s5*s5 + 2000*d5*m5*s2*s3*s4*s5*s5 - 125*c4*c5*m6*s2*s3*s6 + 2000*c2*c5*d5*m6*s4*s4*s5*s6*s6 + 2000*c6*c6*d5*m6*s2*s3*s4*s5*s5 + 2000*d5*m6*s2*s3*s4*s5*s5*s6*s6 + 2000*c2*c5*c6*c6*d5*m6*s4*s4*s5 + 2000*c3*c4*c5*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6))/2000.0;
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_4(){
	double tau=(g*(160*c3*c4*m4*s2 - 160*c2*m4*s4 + 152*c2*c5*c5*m5*s4 + 152*c2*m5*s4*s5*s5 + 125*c2*c4*c5*m6*s6 - 152*c3*c4*c5*c5*m5*s2 - 125*c2*c5*c5*c6*m6*s4 - 2000*c2*c5*c5*d5*m5*s4 - 152*c3*c4*m5*s2*s5*s5 - 125*c2*c6*m6*s4*s5*s5 - 2000*c2*d5*m5*s4*s5*s5 - 2000*c2*d5*m6*s4*s5*s5 + 125*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c4*c5*c5*d5*m5*s2 + 125*c3*c4*c6*m6*s2*s5*s5 + 2000*c3*c4*d5*m5*s2*s5*s5 + 2000*c3*c4*d5*m6*s2*s5*s5 - 2000*c2*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c5*c5*d5*m6*s4*s6*s6 + 125*c3*c5*m6*s2*s4*s6 + 2000*c5*d5*m6*s2*s3*s5 - 2000*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c5*d5*m6*s2*s3*s5*s6*s6 + 2000*c3*c4*c5*c5*c6*c6*d5*m6*s2 + 2000*c3*c4*c5*c5*d5*m6*s2*s6*s6))/2000.0;
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_5(){
	double tau=(g*m6*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5))/16.0;
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_6(){
	double tau=-(m6*(s6*(c2*c4*g + c3*g*s2*s4) - c6*(c5*(c2*g*s4 - c3*c4*g*s2) + g*s2*s3*s5)))/16.0;
	return tau;
}

double LWR4KinematicsDynamics::calculate_gravity_compensation_joint_7(){
	double tau=0;
	return tau;
}



/* IMPEDANCE CONTROL IN CARTESIAN SPACE - remember to call calculateGravitationCompensationTorque before impedanceControlInCartesianSpace */
    std::array<double,7> LWR4KinematicsDynamics::impedanceControlInCartesianSpace(double x_c, double y_c, double z_c, double _v_x, double _v_y, double _v_z, double x_d, double y_d, double z_d) {

			// calculate spring forces - becasue we utilise the impedance control - in cartesian space
			#ifdef DEBUG_SHARED_MEMORY
      	std::cout<<"Impedance Control In Cartesian Space"<<std::endl;
			#endif
// ##################################################################################################

// ##################################################################################################
      // declare equilibrium point - set the desired position of the kinematic chain
      // equilibrium point (x,y,z)

			std::cout<<"TEST - x,y,z="<<x_d<<","<<y_d<<","<<z_d<<std::endl;

      x_desired= x_d;
      y_desired= y_d;
      z_desired= z_d;
      roll_desired = 0;
      pitch_desired = 0;
      yaw_desired = 0;
// ##################################################################################################



// ##################################################################################################

// ##################################################################################################

// ##################################################################################################
			std::cout<<"TEST - x,y,z="<<x_c<<","<<y_c<<","<<z_c<<std::endl;
      x_current=x_c;
      y_current=y_c;
      z_current=z_c;
      //########################################
      roll_current=roll_desired;
      pitch_current=pitch_desired;
      yaw_current=yaw_desired;
// ##################################################################################################

// ##################################################################################################
      /* linear and angular velocities of the end-effector of LWR4+ manipulator */

// #################    in ZY AXIS
      double v_x=_v_x;
      double v_y=_v_x;
      double v_z=_v_y;

      double w_roll=0;
      double w_pitch=0;
      double w_yaw=0;
// ##################################################################################################
			#ifdef DEBUG_SHARED_MEMORY
      	std::cout<<"Current position of end-effector (X,Y,Z)=("<<x_current<<","<<y_current<<","<<z_current<<") angles (ROLL,PITCH,YAW)="<<roll_current<<","<<pitch_current<<","<<yaw_current<<")"<<std::endl;
			#endif
      /* difference_k means the distance between k_desired and k_current along k-axis */
      double difference_x=x_desired-x_current;
      double difference_y=y_desired-y_current;
      double difference_z=z_desired-z_current;
      double difference_roll=0;
      double difference_pitch=0;
      double difference_yaw=0;

			#ifdef DEBUG_SHARED_MEMORY
      	std::cout<<"Difference between desired and current position of end-effector (X,Y,Z)=("<<difference_x<<","<<difference_y<<","<<difference_z<<") angles (ROLL,PITCH,YAW)="<<roll_current<<","<<pitch_current<<","<<yaw_current<<")"<<std::endl;
			#endif

      // stiffness matrix components
      double k11=k, k12=k, k13=k, k14=k, k15=k, k16=k, k21=k, k22=k, k23=k, k24=k, k25=k, k26=k, k31=k, k32=k, k33=k, k34=k, k35=k, k36=k, k41=k, k42=k, k43=k, k44=k, k45=k, k46=k, k51=k, k52=k, k53=k, k54=k, k55=k, k56=k, k61=k, k62=k, k63=k, k64=k, k65=k, k66=k;

      // set up diagonal parameters of stiffness matrix
      k11=k_diag; k22=k_diag; k33=k_diag; k44=k_diag; k55=k_diag; k66=k_diag;

      // damping matrix components
      double d11=d, d12=d, d13=d, d14=d, d15=d, d16=d, d21=d, d22=d, d23=d, d24=d, d25=d, d26=d, d31=d, d32=d, d33=d, d34=d, d35=d, d36=d, d41=d, d42=d, d43=d, d44=d, d45=d, d46=d, d51=d, d52=d, d53=d, d54=d, d55=d, d56=d, d61=d, d62=d, d63=d, d64=d, d65=d, d66=d;
      d11=d_diag; d22=d_diag; d33=d_diag; d44=d_diag; d55=d_diag; d66=d_diag;


      // torque for joint 1
      double t1=k65*(pitch_desired - pitch_current) - d62*v_y - d63*v_z - d65*w_pitch - d64*w_roll - d66*w_yaw - (d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + c1*d3*s2)*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired)) - d61*v_x + k64*(roll_desired - roll_current) - k61*(x_current - x_desired) - k62*(y_current - y_desired) + k66*(yaw_desired - yaw_current) - k63*(z_current - z_desired) - (d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d3*s1*s2)*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired));

      // torque for joint nr 2
      double t2=(c1*(d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + c1*d3*s2) - s1*(d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d3*s1*s2))*(d31*v_x + d32*v_y + d33*v_z + d35*w_pitch + d34*w_roll + d36*w_yaw - k35*(pitch_desired - pitch_current) - k34*(roll_desired - roll_current) + k31*(x_current - x_desired) + k32*(y_current - y_desired) - k36*(yaw_desired - yaw_current) + k33*(z_current - z_desired)) - c1*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired)) + s1*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired)) - c1*(c2*d3 + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4))*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired)) - s1*(c2*d3 + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4))*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired));

      // torque for joint nr 3
      double t3=(s1*s2*(d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))) + c1*s2*(d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))))*(d31*v_x + d32*v_y + d33*v_z + d35*w_pitch + d34*w_roll + d36*w_yaw - k35*(pitch_desired - pitch_current) - k34*(roll_desired - roll_current) + k31*(x_current - x_desired) + k32*(y_current - y_desired) - k36*(yaw_desired - yaw_current) + k33*(z_current - z_desired)) - (c2*(d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))) - c1*s2*(d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4)))*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired)) - (c2*(d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))) + s1*s2*(d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4)))*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired)) - c2*(d61*v_x + d62*v_y + d63*v_z + d65*w_pitch + d64*w_roll + d66*w_yaw - k65*(pitch_desired - pitch_current) - k64*(roll_desired - roll_current) + k61*(x_current - x_desired) + k62*(y_current - y_desired) - k66*(yaw_desired - yaw_current) + k63*(z_current - z_desired)) - c1*s2*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired)) - s1*s2*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired));

      // torque for joint nr 4
      double t4=(c1*c3 - c2*s1*s3)*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired)) - (c3*s1 + c1*c2*s3)*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired)) + ((c3*s1 + c1*c2*s3)*(d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4)) + s2*s3*(d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))))*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired)) - ((c1*c3 - c2*s1*s3)*(d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))) - (d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))))*(c3*s1 + c1*c2*s3))*(d31*v_x + d32*v_y + d33*v_z + d35*w_pitch + d34*w_roll + d36*w_yaw - k35*(pitch_desired - pitch_current) - k34*(roll_desired - roll_current) + k31*(x_current - x_desired) + k32*(y_current - y_desired) - k36*(yaw_desired - yaw_current) + k33*(z_current - z_desired)) + ((c1*c3 - c2*s1*s3)*(d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4)) + s2*s3*(d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))))*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired)) + s2*s3*(d61*v_x + d62*v_y + d63*v_z + d65*w_pitch + d64*w_roll + d66*w_yaw - k65*(pitch_desired - pitch_current) - k64*(roll_desired - roll_current) + k61*(x_current - x_desired) + k62*(y_current - y_desired) - k66*(yaw_desired - yaw_current) + k63*(z_current - z_desired));

      // torque for joint nr 5
      double t5=(d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(c2*c4 + c3*s2*s4))*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired)) - (c2*c4 + c3*s2*s4)*(d61*v_x + d62*v_y + d63*v_z + d65*w_pitch + d64*w_roll + d66*w_yaw - k65*(pitch_desired - pitch_current) - k64*(roll_desired - roll_current) + k61*(x_current - x_desired) + k62*(y_current - y_desired) - k66*(yaw_desired - yaw_current) + k63*(z_current - z_desired)) - (d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))*(c2*c4 + c3*s2*s4) - d7*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)))*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired)) - (s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired)) + (s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired)) - (d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2))*(d31*v_x + d32*v_y + d33*v_z + d35*w_pitch + d34*w_roll + d36*w_yaw - k35*(pitch_desired - pitch_current) - k34*(roll_desired - roll_current) + k31*(x_current - x_desired) + k32*(y_current - y_desired) - k36*(yaw_desired - yaw_current) + k33*(z_current - z_desired));


      // torque for joint nr 6
      double t6=(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired)) + (d7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3))*(d21*v_x + d22*v_y + d23*v_z + d25*w_pitch + d24*w_roll + d26*w_yaw - k25*(pitch_desired - pitch_current) - k24*(roll_desired - roll_current) + k21*(x_current - x_desired) + k22*(y_current - y_desired) - k26*(yaw_desired - yaw_current) + k23*(z_current - z_desired)) + (d7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))*(d31*v_x + d32*v_y + d33*v_z + d35*w_pitch + d34*w_roll + d36*w_yaw - k35*(pitch_desired - pitch_current) - k34*(roll_desired - roll_current) + k31*(x_current - x_desired) + k32*(y_current - y_desired) - k36*(yaw_desired - yaw_current) + k33*(z_current - z_desired)) + (d7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))))*(d11*v_x + d12*v_y + d13*v_z + d15*w_pitch + d14*w_roll + d16*w_yaw - k15*(pitch_desired - pitch_current) - k14*(roll_desired - roll_current) + k11*(x_current - x_desired) + k12*(y_current - y_desired) - k16*(yaw_desired - yaw_current) + k13*(z_current - z_desired)) + (s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(d61*v_x + d62*v_y + d63*v_z + d65*w_pitch + d64*w_roll + d66*w_yaw - k65*(pitch_desired - pitch_current) - k64*(roll_desired - roll_current) + k61*(x_current - x_desired) + k62*(y_current - y_desired) - k66*(yaw_desired - yaw_current) + k63*(z_current - z_desired)) - (s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired));

      // torque for joint nr 7
      double t7=(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))*(d51*v_x + d52*v_y + d53*v_z + d55*w_pitch + d54*w_roll + d56*w_yaw - k55*(pitch_desired - pitch_current) - k54*(roll_desired - roll_current) + k51*(x_current - x_desired) + k52*(y_current - y_desired) - k56*(yaw_desired - yaw_current) + k53*(z_current - z_desired)) - (s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))*(d61*v_x + d62*v_y + d63*v_z + d65*w_pitch + d64*w_roll + d66*w_yaw - k65*(pitch_desired - pitch_current) - k64*(roll_desired - roll_current) + k61*(x_current - x_desired) + k62*(y_current - y_desired) - k66*(yaw_desired - yaw_current) + k63*(z_current - z_desired)) - (c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(d41*v_x + d42*v_y + d43*v_z + d45*w_pitch + d44*w_roll + d46*w_yaw - k45*(pitch_desired - pitch_current) - k44*(roll_desired - roll_current) + k41*(x_current - x_desired) + k42*(y_current - y_desired) - k46*(yaw_desired - yaw_current) + k43*(z_current - z_desired));

			#ifdef DEBUG_SHARED_MEMORY
	      std::cout<<"Calculated torques - impedance control in Cartesian space: "<<std::endl;
	      std::cout<<"[Torque] t1 = "<<t1<<std::endl;
	      std::cout<<"[Torque] t2 = "<<t2<<std::endl;
	      std::cout<<"[Torque] t3 = "<<t3<<std::endl;
	      std::cout<<"[Torque] t4 = "<<t4<<std::endl;
	      std::cout<<"[Torque] t5 = "<<t5<<std::endl;
	      std::cout<<"[Torque] t6 = "<<t6<<std::endl;
	      std::cout<<"[Torque] t7 = "<<t7<<std::endl;
			#endif

			/* Update torque values - add torque values for joints calculated from impedance control in Cartesian space */
      torque[0]+=t1; torque[1]+=t2; torque[2]+=t3; torque[3]+=t4; torque[4]+=t5; torque[5]+=t6; torque[6]+=t7;
			/* END - update torque values */
			return torque;
    }
/* END of impedanceControlInCartesianSpace */

// #############################################################################
// GENERATE SUBSEQUENT TRAJECTORY POSITIONS FOR END-EFFECTOR - in the future prepare special library for planning the movement of end-effector

// Calculates the consequtive end-effector positions. Movement along the circle.
// waiting_iter - how many iterations algorithm waits
// position_iter - current iteration in the circle, 0 means beginning of the circle and MAX_ITERATIONS means the end of the circle
// x - desired x position of end-effector
// y - desired y position of end-effector
// z - desired z position of end-effector
// radius - desired radius of the circle
// origin_i, where i={x,y,z} - the origin of the desired circle (along which the manioulators moves)
void LWR4KinematicsDynamics::calculateNextEndEffectorPosition_Circle_ZY_Plane(int & waiting_iter, int & position_iter, double &x, double &y, double &z, bool & direction, double radius, double origin_x, double origin_y, double origin_z){
  // circle params
  // double radius=0.3;
  // double origin_x=0;
  // double origin_y=0;
	// double origin_z=0.5;

  waiting_iter++; // how many iterations are within a single cycle (waiting a whole cycle to change the desired cartesian position)
  if(waiting_iter>ITERATIONS_IN_ONE_CYCLE){
    waiting_iter=0;
  }
  else{
    return; // just do nothing
  }

  // otherwise calculate new position of end-effector in cartesian space
  if(direction)
    position_iter++;
  else
    position_iter--;

  if(position_iter>MAX_ITERATIONS){
    position_iter--;
    direction=false;
  }
  if(position_iter<0){
    position_iter++;
    direction=true;
  }

  double max_iter=MAX_ITERATIONS;

  // in Z-Y plane
  double theta=(position_iter/max_iter) * 360.0 * 3.14/180.0;
  z= origin_z + radius * sin(theta);
  y= origin_y + radius * cos(theta);

  // double x_new=0;
  std::cout<<"!!!!!ITERATIONS="<<position_iter<<" theta="<<theta<<" EQUILIBRIUM=("<<x<<","<<y<<","<<z<<")"<<std::endl;

  // // ################# sampling period nr 2, i.e. period between calculating consecutive desired end-effector positions ################
  // end_time=std::chrono::system_clock::now();
  // std::cout<<"CALCULATE SAMPLING PERIOD"<<std::endl;
  // std::chrono::duration<double> elapsed_seconds = end_time-start_time;
  // std::cout<< "Elapsed time for a single iteration, i.e. time between calculating a new cartesian position: "<<elapsed_seconds.count()<<std::endl;
  // start_time=end_time;
  // // ####################################################################################
}


// Calculates the consequtive end-effector positions. Movement along the square trajectory.
// waiting_iter - how many iterations algorithm waits
// position_iter - current iteration in the circle, 0 means beginning of the circle and MAX_ITERATIONS means the end of the circle
// x - desired x position of end-effector
// y - desired y position of end-effector
// z - desired z position of end-effector
// radius - desired radius of the circle
// origin_i, where i={x,y,z} - the origin of the desired circle (along which the manioulators moves)
void LWR4KinematicsDynamics::calculateNextEndEffectorPosition_Square_ZY_Plane(int & waiting_iter, int & position_iter, double &x, double &y, double &z, bool & direction, double radius, double origin_x, double origin_y, double origin_z){
  // circle params
  // double radius=0.3;
  // double origin_x=0;
  // double origin_y=0;
	// double origin_z=0.5;

  waiting_iter++; // how many iterations are within a single cycle (waiting a whole cycle to change the desired cartesian position)
  if(waiting_iter>ITERATIONS_IN_ONE_CYCLE){
    waiting_iter=0;
  }
  else{
    return; // just do nothing
  }

  // otherwise calculate new position of end-effector in cartesian space
  if(direction)
    position_iter++;
  else
    position_iter--;

  if(position_iter>MAX_ITERATIONS){
    position_iter--;
    direction=false;
  }
  if(position_iter<0){
    position_iter++;
    direction=true;
  }

  double max_iter=MAX_ITERATIONS;

	// in Z-Y plane
	double current_value=position_iter/max_iter;
	// check along which wall is currently the end-effector moving
	if(current_value<0.25){
		// north wall
		// move along y axis
		z = origin_z+radius;
		y = origin_y-radius + current_value*4*2*radius;
	}
	else if(current_value>=0.25 && current_value<0.5){
		// east wall
		// move along z axis
		y = origin_y+radius;
		z = origin_z+radius - (current_value-0.25)*4*2*radius;

	}
	else if(current_value>=0.5 && current_value<0.75){
		// south wall
		// move along y axis
		z = origin_z-radius;
		y = origin_y+radius - (current_value-0.5)*4*2*radius;
	}
	else if(current_value>=0.75 && current_value<1){
		// west wall
		// move along z axis
		y = origin_y+radius;
		z = origin_z-radius + (current_value-0.75)*4*2*radius;
	}

  // double x_new=0;
  std::cout<<"!!!!!ITERATIONS="<<current_value<<" EQUILIBRIUM=("<<x<<","<<y<<","<<z<<")"<<std::endl;
}
