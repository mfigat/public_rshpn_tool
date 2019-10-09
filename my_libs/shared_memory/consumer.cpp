// CONSUMER
#include "shared_memory.h"

int main(){

  int i = function(5);
  std::cout<<i<<std::endl;



  // #############################################################################################
  // struct lwr4_joints type
  // #############################################################################################

  struct lwr4_joints tmp;
  SharedMemory<struct lwr4_joints> sm_consumer("shared_memory_float", SharedMemoryType::Consumer);

  while(true){
	  // tmp=sm_consumer.readSynchronously();
	    tmp=sm_consumer.readAsynchronously();
#ifdef DEBUG_SHARED_MEMORY
      std::cout << "Original Value:"<<std::endl;
      for(int i=0;i<7;i++)
	     std::cout <<"Joint["<<i<<"]="<<tmp._joints[i]<<std::endl;
#endif
	    sleep(1);
   }



   // #############################################################################################
   // float type
   // #############################################################################################
/*
   float tmp;
   SharedMemory<float> sm_consumer("shared_memory_float", SharedMemoryType::Consumer);
   while(true){
// #################################################################
     // CONSUMER - communication options:
// #################################################################
 	   tmp=sm_consumer.readSynchronously();                         // producer-consumer -> block - block
 	   //tmp=sm_consumer.readAsynchronously();                        // producer-consumer -> non-block - non-block
     //tmp=sm_consumer.readAsynchronouslyWriteSynchronously();      // producer-consumer -> block - non-block
     //tmp=sm_consumer.readSynchronouslyWriteAsynchronously();      // producer-consumer -> non-block - block
// #################################################################

#ifdef DEBUG_SHARED_MEMORY
       std::cout << "Original Value: "<<tmp<<" Sqrt: "<< sqrt(tmp) << "\n";
#endif
 	    sleep(1);
    }
  */



   // #############################################################################################
   // struct maxym type
   // #############################################################################################
   /*
   struct maxym tmp;
   SharedMemory<struct maxym> sm_consumer("shared_memory_float", SharedMemoryType::Consumer);
   while(true){
 	   tmp=sm_consumer.readSynchronously();
 	   // tmp=sm_consumer.readAsynchronously();
#ifdef DEBUG_SHARED_MEMORY
       std::cout << "Original Value:"<<tmp.y<<" Sqrt: "<< sqrt(tmp.y) << "\n";
#endif
 	    sleep(1);
    }
    */


   return 0;
}
