#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>

#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>

#include <boost/version.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include <iostream>

/**
  Define directive DEBUG_SHARED_MEMORY to print values written and read - to/from shared memory,
  otherwise comment the #define DEBUG_SHARED_MEMORY 0 line
*/
#define DEBUG_SHARED_MEMORY 1

#define SUCCESS 0
#define ERROR -1

using namespace boost::interprocess;

/**

struct for shared memory - with float

*/

struct maxym{
	int x;
	float y;
};

struct lwr4_joints{
	double _joints[7];
};

struct lwr4_end_effector_position{
	double x_desired;
	double y_desired;
	double z_desired;
	double roll_desired;
	double pitch_desired;
	double yaw_desired;
};


struct lwr4_kinematics_params{
	/* End effector's frame position and velocity */
	double x_current;
	double y_current;
	double z_current;
	double roll_current;
	double pitch_current;
	double yaw_current;

	double v_x;
	double v_y;
	double v_z;
	double w_roll;
	double w_pitch;
	double w_yaw;

	double theta1;
	double theta2;
	double theta3;
	double theta4;
	double theta5;
	double theta6;
	double theta7;
};



// data type in shared memory buffer - float
template <typename T>
struct shared_memory_buffer_float {
/**
	sempahores assurring that two processes do not have an access to the same shared memory buffer at the same time, there are two semaphores:
    	writer - if is 1 then the writer may start writing to the shared memory,
	reader - if is 1 then the reader may start reading,
*/
	// constructor
	shared_memory_buffer_float<T>(): writer(1), reader(1){}

 	// semaphores
	interprocess_semaphore writer, reader;

 	// message which is sent through shared memory
 	T msg;
};




enum SharedMemoryType{
Producer=0, Consumer=1
};


// Shared memory class
template <typename T>
class SharedMemory{

	public:

		/* Constructor without parameters */
		SharedMemory<T>(){
		};

		/* Constructor with shared memory name and type of the process - either producer of the message or consumer */
		SharedMemory<T>(std::string name, int type):_type(type), _sharedMemoryName(name.c_str()){
			if(_type==SharedMemoryType::Consumer){ // if
				openSharedMemory();
			}
			else if(_type==SharedMemoryType::Producer){
				openOrCreateSharedMemory();
			}
			else{
				perror("None of SharedMemory type");
			}
		};
		~SharedMemory(){
		      remove(_sharedMemoryName);
		};
		int openSharedMemory();
		int openOrCreateSharedMemory();

// #################################
// read synchronously
		T readSynchronously();
// read synchronously
		void writeSynchronously(T);
// #################################


// #################################
// read asynchronously
		T readAsynchronously();
// write asynchronously
		void writeAsynchronously(T);
// #################################

// #################################
// write (synchronously) - read (asynchronously)
		T readAsynchronouslyWriteSynchronously();
		void writeSynchronouslyReadAsynchronously(T);
// #################################

// #################################
// write (asynchronously) - read (synchronously)
		T readSynchronouslyWriteAsynchronously();
		void writeAsynchronouslyReadSynchronously(T);
// #################################


	private:
		int _type; // indicates whether it is a producer or consumer
		const char * _sharedMemoryName; // indicates the shared memory name

		void * _addr;
		shared_memory_object _shm;
		mapped_region _region;

		// add here a template
		shared_memory_buffer_float<T> * _sharedMemory; // indicates the pointer to the shared_memory_buffer_float
};

void testPrint(std::string str);
int function(int);
