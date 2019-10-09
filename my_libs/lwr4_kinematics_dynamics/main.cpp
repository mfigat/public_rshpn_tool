
// PRODUCER
#include "lwr4_kinematics_dynamics.h"
#include "../shared_memory/shared_memory.h"

// ################
#include <chrono> // needed to get the current time
#include <ctime>
// ################



#define PERIOD_BETWEEN_WHILE_ITERATIONS 1000 //1000 // 500 ok // 1000 ok

// period sample
std::chrono::time_point<std::chrono::system_clock> start_time=std::chrono::system_clock::now();
std::chrono::time_point<std::chrono::system_clock> end_time=std::chrono::system_clock::now();


void calculateEndEffectorPosition(int & waiting_iter, int & position_iter, double &x, double &y, double &z, bool & direction){
  // circle params
  double radius=0.3;
  double origin_x=0;
  double origin_y=0;
  double origin_z=0.5;

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

  double x_new=0;
  std::cout<<"!!!!!ITERATIONS="<<position_iter<<" theta="<<theta<<" EQUILIBRIUM=("<<x_new<<","<<y<<","<<z<<")"<<std::endl;

  // ################# sampling period nr 2, i.e. period between calculating consecutive desired end-effector positions ################
  end_time=std::chrono::system_clock::now();
  std::cout<<"CALCULATE SAMPLING PERIOD"<<std::endl;
  std::chrono::duration<double> elapsed_seconds = end_time-start_time;
  std::cout<< "Elapsed time for a single iteration, i.e. time between calculating a new cartesian position: "<<elapsed_seconds.count()<<std::endl;
  start_time=end_time;
  // ####################################################################################

}


int main(){
  LWR4KinematicsDynamics lwr=LWR4KinematicsDynamics(0, 30, 0, 10); // 0 20 0 10 - bylo ok

  // ########################




  //int i = lwr.function(4);
  //std::cout<<"Main function "<<i<<std::endl;

  //lwr.calculateGravitationCompensationTorque(0,1.57,0,0,0,0,0);

  //lwr.impedanceControlInCartesianSpace(0,0,1.3, 1,1,0.5, 0,0,0);


	std::cout<<"TEST shared_memory_lwr4_kinematics_params"<<std::endl;
  struct lwr4_kinematics_params tmp;
  SharedMemory<struct lwr4_kinematics_params> sm_consumer("shared_memory_lwr4_kinematics_params", SharedMemoryType::Consumer);

	std::cout<<"TEST shared_memory_torque"<<std::endl;
  struct lwr4_joints msg_tmp;
  SharedMemory<struct lwr4_joints> sm_producer("shared_memory_torque", SharedMemoryType::Producer);

  std::array<double,7> torque_grav, torque_impedance;

  /* Desired positions */
  double x_d=0.56, y_d=0.17, z_d=0.8;



  int _iterations2=0;
  int _iterations=0;



  bool _flag=true;

  while(true){ // endless while loop

    /* CLEAN torque_grav and torque_impedance */
    for(int i=0;i<7;i++){
      torque_grav[i]=0;
      torque_impedance[i]=0;
    }

    // ################# sampling period nr 1 ################
    std::cout<<"CALCULATE SAMPLING PERIOD"<<std::endl;
    std::chrono::duration<double> elapsed_time_between_iterations = std::chrono::system_clock::now()-end_time;
    std::cout<< "Elapsed between iterations: "<<elapsed_time_between_iterations.count()<<std::endl;
    end_time=std::chrono::system_clock::now();
    // ######################################################

    //calculateEndEffectorPosition(_iterations2, _iterations, x_d, y_d, z_d, _flag);

    LWR4KinematicsDynamics::calculateNextEndEffectorPosition_Circle_ZY_Plane(_iterations2, _iterations, x_d, y_d, z_d, _flag, 0.3, 0, 0, 0.5);





    /* ######## Read data from shared memory ######## */
     //tmp=sm_consumer.readSynchronously();
     tmp=sm_consumer.readAsynchronously();
     // std::cout <<"Theta1="<<tmp.theta1<<std::endl;
     // std::cout <<"Theta2="<<tmp.theta2<<std::endl;
     // std::cout <<"Theta3="<<tmp.theta3<<std::endl;
     // std::cout <<"Theta4="<<tmp.theta4<<std::endl;
     // std::cout <<"Theta5="<<tmp.theta5<<std::endl;
     // std::cout <<"Theta6="<<tmp.theta6<<std::endl;
     // std::cout <<"Theta7="<<tmp.theta7<<std::endl;

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

      usleep(PERIOD_BETWEEN_WHILE_ITERATIONS);
   }



  // while(true){
  //   // read data from shared memory
  //
  // }
}
