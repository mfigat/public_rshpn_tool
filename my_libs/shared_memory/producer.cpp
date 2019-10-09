// PRODUCER
#include "shared_memory.h"

int main(){
  int i = function(10);
  std::cout<<i<<std::endl;

srand(time(NULL));


// #############################################################################################
// struct lwr4_joints type
// #############################################################################################

struct lwr4_joints msg_tmp;

SharedMemory<struct lwr4_joints> sm_producer("shared_memory_torque", SharedMemoryType::Producer);
double torque=0;
double delta_torque=0.001;
while(true){
  for(int i=0;i<7;i++){
	   msg_tmp._joints[i] = torque;
  }
  torque+=delta_torque;
#ifdef DEBUG_SHARED_MEMORY
  std::cout<<"Writes message: "<<std::endl;
  for(int i=0;i<7;i++)
	 std::cout<<"Joint["<<i<<"]="<<msg_tmp._joints[i]<<std::endl;
#endif
	//sm_producer.writeSynchronously(msg_tmp);
	sm_producer.writeAsynchronously(msg_tmp);
	//sleep(1);
}


// #############################################################################################
// float type
// #############################################################################################
/*
float msg_tmp;

SharedMemory<float> sm_producer("shared_memory_float", SharedMemoryType::Producer);

while(true){
  msg_tmp = rand() % 360;
#ifdef DEBUG_SHARED_MEMORY
  std::cout<<"Writes message: "<<msg_tmp<<std::endl;
#endif

// #################################################################
  // PRODUCER - communication options:
// #################################################################
	//sm_producer.writeSynchronously(msg_tmp);                       // producer-consumer -> block - block
	sm_producer.writeAsynchronously(msg_tmp);                      // producer-consumer -> nonblock - nonblock
  //sm_producer.writeSynchronouslyReadAsynchronously(msg_tmp);     // producer-consumer -> block - nonblock
  //sm_producer.writeAsynchronouslyReadSynchronously(msg_tmp);     // producer-consumer -> nonblock - block
// #################################################################

	sleep(1);
}
*/

// #############################################################################################

// #############################################################################################
// struct maxym type
// #############################################################################################
/*
struct maxym msg_tmp;
SharedMemory<struct maxym> sm_producer("shared_memory_float", SharedMemoryType::Producer);

while(true){
  msg_tmp.y = rand() % 360;
#ifdef DEBUG_SHARED_MEMORY
  std::cout<<"Writes message: "<<msg_tmp.y<<std::endl;
#endif
	sm_producer.writeSynchronously(msg_tmp);
	//sm_producer.writeAsynchronously(msg_tmp);
	sleep(2);
}
*/
// #############################################################################################

return 0;
}
