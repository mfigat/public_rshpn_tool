#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <array>


//#include <ignition/math/Vector3.hh>

/**
  Define directive DEBUG_SHARED_MEMORY to print values written and read - to/from shared memory,
  otherwise comment the #define DEBUG_SHARED_MEMORY 0 line
*/
//#define DEBUG_SHARED_MEMORY 1

#define SUCCESS 0
#define ERROR -1

/* Gravitation */
#define gravity_constant 9.81

/* MANIPULATOR MASSES */
#define MASS_1 2
#define MASS_2 2
#define MASS_3 2
#define MASS_4 2
#define MASS_5 2
#define MASS_6 0.2
#define MASS_7 0.002

/* DENAVIT-HARTENBERG parameter d_i */
#define D1 0.31   // d1=0.31m
#define D3 0.4    // d3=0.4m
#define D5 0.39   // d5=0.39m
#define D7 0.078  // d7=0.078m


/* FOR TRAJECTORY GENERATION */
#define MAX_ITERATIONS 720 // 360 ok // 720 dla symulacji z shared_memory - ok
#define ITERATIONS_IN_ONE_CYCLE 10 // 100 ok // 100 dla symulacji z shared_memory - ok




class LWR4KinematicsDynamics{

	public:

		LWR4KinematicsDynamics(double _k, double _k_diag, double d, double d_diag);
		~LWR4KinematicsDynamics(){}

		int function(int i);

		/* calculate torques compensating the gravitation */
		std::array<double,7> calculateGravitationCompensationTorque(double _theta1, double _theta2, double _theta3, double _theta4, double _theta5, double _theta6, double _theta7);

		/* calculate torques - from cartesian impedance control */
		std::array<double,7> impedanceControlInCartesianSpace(double x_c, double y_c, double z_c, double _v_x, double _v_y, double _v_z, double x_d, double y_d, double z_d);

		/* calculate subsequent positions of end-effector - Circle */
		static void calculateNextEndEffectorPosition_Circle_ZY_Plane(int & waiting_iter, int & position_iter, double &x, double &y, double &z, bool & direction, double radius, double origin_x, double origin_y, double origin_z);

		/* calculate subsequent positions of end-effector - Square */
		static void calculateNextEndEffectorPosition_Square_ZY_Plane(int & waiting_iter, int & position_iter, double &x, double &y, double &z, bool & direction, double radius, double origin_x, double origin_y, double origin_z);

	private:
		int _x;

		/* JOINTS POSITIONS IN JOINT SPACE */
		double theta1;
		double theta2;
		double theta3;
		double theta4;
		double theta5;
		double theta6;
		double theta7;
		/* JOINTS - END

		/* MANIPULATOR LINKS - MASSES */
		double m1, m2, m3, m4, m5, m6, m7;
		/* END MASSES */

		/* DENAVIT-HARTENBERG parameters of LWR4+ manipulator */
		double d1, d3, d5, d7;
		/* END D-H parameters */

		/* SINUSES and COSINUSES */
		double c1, c2, c3, c4, c5, c6, c7;
		double s1, s2, s3, s4, s5, s6, s7;
		/* END SINUS and COSINUSES */

		/* GRAVITATION */
		double g;
		/* END GRAVITATION */

		/* Array of torques for LWR4+ manipulator */
		std::array<double,7> torque;
		/* END array of torque */

		/* Current end-effector velocities */
		double v_x, v_y, v_z; 						// linear Velocities
		double w_roll, w_pitch, w_yaw; 		// angular velocities
		/* END - end-effector velocities */

		/* Current position of end-effector frame */
		double x_current, y_current, z_current; 					// Current position in Cartesian space of end-effector frame
		double roll_current, pitch_current, yaw_current; 	// orientation of the end-effector frame
		/* END - Current position of end-effector frame */

		/* Desired position of end-effector frame */
		double x_desired, y_desired, z_desired; 					// Desired position in Cartesian space of end-effector frame
		double roll_desired, pitch_desired, yaw_desired; 	// orientation of the end-effector frame
		/* END - Desired position of end-effector frame */

		/* Parameters for stiffness matrix */
		double k;  				// value for all elements in stiffness matrix except the elements on the diagonal
		double k_diag;		// value for all elements of stiffness matrix, which are located on the diagonal of the matrix
		/* END - Parameters for stiffness matrix */

		/* Parameters for damping matrix */
		double d;  				// value for all elements in damping matrix except the elements on the diagonal
		double d_diag;		// value for all elements of damping matrix, which are located on the diagonal of the matrix
		/* END - Parameters for damping matrix */

		/* PRIVATE METHODS */

		/* Methods required to calculate gravitation compensation */
		double calculate_gravity_compensation_joint_7();
		double calculate_gravity_compensation_joint_6();
		double calculate_gravity_compensation_joint_5();
		double calculate_gravity_compensation_joint_4();
		double calculate_gravity_compensation_joint_3();
		double calculate_gravity_compensation_joint_2();
		double calculate_gravity_compensation_joint_1();
		/* END - Gravity methods compensation - for each joint */


};
