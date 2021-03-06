// ###################################################################################################
/* Choose either GAZEBO_CALCULATIONS or EXTERNAL_CALCULATIONS */
//#define GAZEBO_CALCULATIONS /* calculation done in gazebo, no shared memory utilised */
#define EXTERNAL_CALCULATIONS /* calculation done in external process, data sent through share memory */
// ###################################################################################################

// ##############################################
#define CALCULATE_SAMPLING_PERIOD /* An information whether the sampling period has to be calculated - leave it if you want to calculate sampling period otherwise comment it */
// ##############################################
//#define PRINT_DEBUG_INFO /* if defined all cout will be printed otherwise all information will not be printed to the console */
// ##########################################

#include <iostream>
#include <fstream>

// ################
#include <chrono> // needed to get the current time
#include <ctime>
// ################

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// Communication
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Kuka communication messages - my own
//#include "msgs/kuka_joints.pb.h"
#include "kuka_joints.pb.h"

// lwr4_kinematics_dynamics - moja biblioteka
#include "../../../../my_libs/lwr4_kinematics_dynamics/lwr4_kinematics_dynamics.h"

// shared memory - moja biblioteka
#include "../../../../my_libs/shared_memory/shared_memory.h"

//#define gravity_constant 9.80665 // gravity acceleration - from General Conference on Weights and Measures - 1901 year
#define gravity_constant 9.81 // gravity acceleration - option 2

#define L1 0.2 // według gazebo 0.2005 // 0.2 // l1=0.2m
#define L2 0.2 // l2=0.2m
#define L3 0.2 // l3=0.2m
#define L4 0.195 // l4=0.195m
#define L5 0.195 // według gazebo  0.2 //0.195 // l5=0.195m

#define D1 0.31   // d1=0.31m
#define D3 0.4    // d3=0.4m
#define D5 0.39   // d5=0.39m
#define D7 0.078  // d7=0.078m



#define OPTION_1
//#define OPTION_2



#ifdef GAZEBO_CALCULATIONS
  // for GAZEBO_CALCULATIONS
  #define MAX_ITERATIONS 360
  #define ITERATIONS_IN_ONE_CYCLE 50
#endif // GAZEBO_CALCULATIONS

#ifdef EXTERNAL_CALCULATIONS
  // for EXTERNAL_CALCULATIONS
  #define MAX_ITERATIONS 360
  #define ITERATIONS_IN_ONE_CYCLE 50
#endif // EXTERNAL_CALCULATIONS

//std::array<double,7> equilibrium_global={0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};

namespace gazebo
{
  class ModelKukaLwr : public ModelPlugin
  {




    // pointer to Int message
    typedef const boost::shared_ptr<const gazebo::msgs::Any> AnyPtr;
    typedef const boost::shared_ptr<const kuka_joints_msgs::KukaJoints> KukaJointsPtr;

    // Load function
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Stores the pointer to the model
      this->model_ = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      // connects OnUpadate method to the world update start signal
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelKukaLwr::OnUpdate, this));

        #ifdef PRINT_DEBUG_INFO
          std::cout << "plugin loaded" << std::endl;
        #endif

// ####################################################
    // test
    // set equilibrium to base position
    //equilibrium= {0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}; // equilibrium joint position
    equilibrium= {0, 0, 0, 1.57, 0, 0, 0}; // equilibrium joint position

    equilibrium_x=0.56;
    equilibrium_y=0.17;
    equilibrium_z=0.8;
    equilibrium_roll=0;
    equilibrium_pitch=0;
    equilibrium_yaw=0;

    eq_0_step=0.001; //
    eq_3_step=0.001;
    kinematic_chain_index=0;

// ####################################################
        const std::string name = "lwr";
        for (int i = 0; i < 7; ++i) {
            std::string joint_name = std::string("lwr::") + name + "_arm_" + std::to_string(i) + "_joint";
            joints_.push_back(model_->GetJoint(joint_name)); // add joint to joints_ vector (added joint is found from the model based on its name)
        }

        for (int i = 0; i < 7; ++i) {
            std::string link_name = std::string("lwr::") + name + "_arm_" + std::to_string(i+1) + "_link";
            links_.push_back(model_->GetLink(link_name)); // analogously like in joints
        }

// ############################################
    // subscribe to specific topic, e.g. ~/test/maxym
    // create node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node);
    node->Init();
    // listen to gazebo ~/test/maxym topic
    sub=node->Subscribe("~/test/maxym", &ModelKukaLwr::subscribe_callback_function, this);
// ###############################################
// subscribe to kuka_joints topic
    gazebo::transport::NodePtr node_kuka_joints(new gazebo::transport::Node);
    node_kuka_joints->Init();
    sub_kuka_joints=node_kuka_joints->Subscribe("~/kuka_joints", &ModelKukaLwr::subscribe_callback_function_kuka_joints, this);


    _iterations=0;
    _iterations2=0;
    _flag=true;

    // add to vector the base position
    saveBasePoseToFile();



    // tests

    // shared memory test
    #ifdef EXTERNAL_CALCULATIONS /* if data are calculated in external process and transfered through shared memory */
			std::cout<<"SetUpSharedMemory"<<std::endl;
      setUpSharedMemory();
    #endif


    // ###########################
    #ifdef CALCULATE_SAMPLING_PERIOD

      start_time=std::chrono::system_clock::now();

    #endif // CALCULATE_SAMPLING_PERIOD

    // #############################


// ############################################

    } // end of function Load

#ifdef EXTERNAL_CALCULATIONS
    /* Set up shared memory */
    void setUpSharedMemory(){
      //#ifdef PRINT_DEBUG_INFO
      std::cout<<"[EXTERNAL_CALCULATIONS] - setUpSharedMemory"<<std::endl;
      //#endif
      //shm_producer= new SharedMemory<float>("shared_memory_float", SharedMemoryType::Producer); /* shared memory - test */
      shm_torque_consumer= new SharedMemory<struct lwr4_joints>("shared_memory_torque", SharedMemoryType::Consumer); /* Consumer of torque data received from shared memory */
      shm_parameters_producer= new SharedMemory<struct lwr4_kinematics_params>("shared_memory_lwr4_kinematics_params", SharedMemoryType::Producer); /* Consumer of torque data received from shared memory */
    }

#endif


    // set forces to joints based on given torques
    void setForces(const std::array<double, 7 > &t) {
        for (int i=0; i<joints_.size(); i++) {
            joints_[i]->SetForce(0, t[i]); // axis 0 jest default
            //std::cout<<"Torque["<<i<<"]="<<t[i]<<std::endl;
        }
    } // end of function setForces

    double gravity_compensation_joint_7(){
      double tau=0;
      return tau;
    }

    double gravity_compensation_joint_6(){

      double g=gravity_constant;

      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d7= D7, d5 = D5, d3=D3, d1=D1;

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);

      double tau=0;

#ifdef OPTION_1
      // wersja nr 1
      /*
      tau=-d7*(c7*m7*(s7*(s5*(c2*g*s4 - c3*c4*g*s2) - c5*g*s2*s3) + c7*(s6*(c2*c4*g + c3*g*s2*s4) - c6*(c5*(c2*g*s4 - c3*c4*g*s2) + g*s2*s3*s5))) + m7*s7*(s7*(s6*(c2*c4*g + c3*g*s2*s4) - c6*(c5*(c2*g*s4 - c3*c4*g*s2) + g*s2*s3*s5)) - c7*(s5*(c2*g*s4 - c3*c4*g*s2) - c5*g*s2*s3)));
      */
      //tau=-(m6*(s6*(c2*c4*g + c3*g*s2*s4) - c6*(c5*(c2*g*s4 - c3*c4*g*s2) + g*s2*s3*s5)))/16.0;
      tau=-(m6*(s6*(c2*c4*g + c3*g*s2*s4) - c6*(c5*(c2*g*s4 - c3*c4*g*s2) + g*s2*s3*s5)))/16.0;
#endif
      return tau;
    }

    /////////////////////////////////////////////

    double gravity_compensation_joint_5(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;
      double g=gravity_constant;
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5;

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);

      double tau=0;

#ifdef OPTION_1
      // wersja nr 1
      /*

      */
      //tau=(g*m6*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5))/16.0;

      tau=(g*m6*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5))/16.0;

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"tau5="<<tau<<std::endl;
      #endif
#endif
      return tau;
    }

    double gravity_compensation_joint_4(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;
      double g=gravity_constant;
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);

      double tau=0;

#ifdef OPTION_1
      // wersja nr 1
      /*

      */
//      tau=(g*(140*c3*c4*m4*s2 - 140*c2*m4*s4 + 152*c2*c5*c5*m5*s4 + 152*c2*m5*s4*s5*s5 + 125*c2*c4*c5*m6*s6 - 152*c3*c4*c5*c5*m5*s2 - 125*c2*c5*c5*c6*m6*s4 - 2000*c2*c5*c5*d5*m5*s4 - 152*c3*c4*m5*s2*s5*s5 - 125*c2*c6*m6*s4*s5*s5 - 2000*c2*d5*m5*s4*s5*s5 - 2000*c2*d5*m6*s4*s5*s5 + 125*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c4*c5*c5*d5*m5*s2 + 125*c3*c4*c6*m6*s2*s5*s5 + 2000*c3*c4*d5*m5*s2*s5*s5 + 2000*c3*c4*d5*m6*s2*s5*s5 - 2000*c2*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c5*c5*d5*m6*s4*s6*s6 + 125*c3*c5*m6*s2*s4*s6 + 2000*c5*d5*m6*s2*s3*s5 - 2000*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c5*d5*m6*s2*s3*s5*s6*s6 + 2000*c3*c4*c5*c5*c6*c6*d5*m6*s2 + 2000*c3*c4*c5*c5*d5*m6*s2*s6*s6))/2000.0;

      //tau=(g*(180*c3*c4*m4*s2 - 180*c2*m4*s4 + 152*c2*c5*c5*m5*s4 + 152*c2*m5*s4*s5*s5 + 125*c2*c4*c5*m6*s6 - 152*c3*c4*c5*c5*m5*s2 - 125*c2*c5*c5*c6*m6*s4 - 2000*c2*c5*c5*d5*m5*s4 - 152*c3*c4*m5*s2*s5*s5 - 125*c2*c6*m6*s4*s5*s5 - 2000*c2*d5*m5*s4*s5*s5 - 2000*c2*d5*m6*s4*s5*s5 + 125*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c4*c5*c5*d5*m5*s2 + 125*c3*c4*c6*m6*s2*s5*s5 + 2000*c3*c4*d5*m5*s2*s5*s5 + 2000*c3*c4*d5*m6*s2*s5*s5 - 2000*c2*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c5*c5*d5*m6*s4*s6*s6 + 125*c3*c5*m6*s2*s4*s6 + 2000*c5*d5*m6*s2*s3*s5 - 2000*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c5*d5*m6*s2*s3*s5*s6*s6 + 2000*c3*c4*c5*c5*c6*c6*d5*m6*s2 + 2000*c3*c4*c5*c5*d5*m6*s2*s6*s6))/2000.0;

      tau=(g*(160*c3*c4*m4*s2 - 160*c2*m4*s4 + 152*c2*c5*c5*m5*s4 + 152*c2*m5*s4*s5*s5 + 125*c2*c4*c5*m6*s6 - 152*c3*c4*c5*c5*m5*s2 - 125*c2*c5*c5*c6*m6*s4 - 2000*c2*c5*c5*d5*m5*s4 - 152*c3*c4*m5*s2*s5*s5 - 125*c2*c6*m6*s4*s5*s5 - 2000*c2*d5*m5*s4*s5*s5 - 2000*c2*d5*m6*s4*s5*s5 + 125*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c4*c5*c5*d5*m5*s2 + 125*c3*c4*c6*m6*s2*s5*s5 + 2000*c3*c4*d5*m5*s2*s5*s5 + 2000*c3*c4*d5*m6*s2*s5*s5 - 2000*c2*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c5*c5*d5*m6*s4*s6*s6 + 125*c3*c5*m6*s2*s4*s6 + 2000*c5*d5*m6*s2*s3*s5 - 2000*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c5*d5*m6*s2*s3*s5*s6*s6 + 2000*c3*c4*c5*c5*c6*c6*d5*m6*s2 + 2000*c3*c4*c5*c5*d5*m6*s2*s6*s6))/2000.0;

#endif

      return tau;
    }

    double gravity_compensation_joint_3(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;


      double g=gravity_constant;
      double m3 = links_[2]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);
      double tau=0;

#ifdef OPTION_1
      // wersja nr 1
      /*

      */
      //tau=-(g*(140*m4*s2*s3*s4 - 120*c3*m3*s2 + 120*c3*c4*c4*m4*s2 + 120*c3*m4*s2*s4*s4 - 152*c5*c5*m5*s2*s3*s4 - 152*m5*s2*s3*s4*s5*s5 - 2000*c2*c5*d5*m6*s4*s4*s5 - 125*c3*c4*c4*m6*s2*s5*s6 + 125*c5*c5*c6*m6*s2*s3*s4 + 2000*c5*c5*d5*m5*s2*s3*s4 + 2000*c5*c5*d5*m6*s2*s3*s4 - 125*c3*m6*s2*s4*s4*s5*s6 + 125*c6*m6*s2*s3*s4*s5*s5 + 2000*d5*m5*s2*s3*s4*s5*s5 - 125*c4*c5*m6*s2*s3*s6 + 2000*c2*c5*d5*m6*s4*s4*s5*s6*s6 + 2000*c6*c6*d5*m6*s2*s3*s4*s5*s5 + 2000*d5*m6*s2*s3*s4*s5*s5*s6*s6 + 2000*c2*c5*c6*c6*d5*m6*s4*s4*s5 + 2000*c3*c4*c5*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6))/2000.0;

      tau=-(g*(160*m4*s2*s3*s4 - 120*c3*m3*s2 + 120*c3*c4*c4*m4*s2 + 120*c3*m4*s2*s4*s4 - 152*c5*c5*m5*s2*s3*s4 - 152*m5*s2*s3*s4*s5*s5 - 2000*c2*c5*d5*m6*s4*s4*s5 - 125*c3*c4*c4*m6*s2*s5*s6 + 125*c5*c5*c6*m6*s2*s3*s4 + 2000*c5*c5*d5*m5*s2*s3*s4 + 2000*c5*c5*d5*m6*s2*s3*s4 - 125*c3*m6*s2*s4*s4*s5*s6 + 125*c6*m6*s2*s3*s4*s5*s5 + 2000*d5*m5*s2*s3*s4*s5*s5 - 125*c4*c5*m6*s2*s3*s6 + 2000*c2*c5*d5*m6*s4*s4*s5*s6*s6 + 2000*c6*c6*d5*m6*s2*s3*s4*s5*s5 + 2000*d5*m6*s2*s3*s4*s5*s5*s6*s6 + 2000*c2*c5*c6*c6*d5*m6*s4*s4*s5 + 2000*c3*c4*c5*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2000*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6))/2000.0;
#endif
      return tau;
    }

    double gravity_compensation_joint_2(){


      double g=gravity_constant;

      double m2 = links_[1]->GetInertial()->Mass(); // because 1-th link in links_ is in fact 2-th link of the lwr4+ manipulator
      double m3 = links_[2]->GetInertial()->Mass(); // because 2-th link in links_ is in fact 3-th link of the lwr4+ manipulator
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3, l2=L2;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);
      double tau=0;

#ifdef OPTION_1
      // wersja nr 1
      /*

      */
      /*
      tau=-(g*(140*m2*s2 - 140*c3*c3*m3*s2 - 140*m3*s2*s3*s3 - 120*c2*m3*s3 - 140*c2*c3*m4*s4 + 120*c2*c4*c4*m4*s3 + 140*c3*c3*c4*m4*s2 + 2000*c3*c3*d3*m3*s2 + 120*c2*m4*s3*s4*s4 + 140*c4*m4*s2*s3*s3 + 2000*d3*m3*s2*s3*s3 + 2000*d3*m4*s2*s3*s3 - 152*c3*c3*c4*c5*c5*m5*s2 + 2000*c3*c3*c4*c4*d3*m4*s2 - 152*c4*c5*c5*m5*s2*s3*s3 - 152*c3*c3*c4*m5*s2*s5*s5 + 2000*c3*c3*d3*m4*s2*s4*s4 + 2000*c3*c3*d3*m5*s2*s4*s4 + 2000*c5*c5*d3*m5*s2*s3*s3 + 2000*c5*c5*d3*m6*s2*s3*s3 - 152*c4*m5*s2*s3*s3*s5*s5 + 2000*d3*m5*s2*s3*s3*s5*s5 + 152*c2*c3*c5*c5*m5*s4 + 152*c2*c3*m5*s4*s5*s5 + 2000*c3*c3*d3*m6*s2*s4*s4*s6*s6 + 2000*c6*c6*d3*m6*s2*s3*s3*s5*s5 + 2000*d3*m6*s2*s3*s3*s5*s5*s6*s6 - 125*c2*c3*c5*c5*c6*m6*s4 - 2000*c2*c3*c5*c5*d5*m5*s4 - 125*c2*c3*c6*m6*s4*s5*s5 - 2000*c2*c3*d5*m5*s4*s5*s5 - 2000*c2*c3*d5*m6*s4*s5*s5 - 125*c2*c4*c4*m6*s3*s5*s6 + 125*c3*c3*c5*m6*s2*s4*s6 - 125*c2*m6*s3*s4*s4*s5*s6 + 125*c5*m6*s2*s3*s3*s4*s6 + 125*c3*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c3*c4*c5*c5*d5*m5*s2 + 125*c4*c5*c5*c6*m6*s2*s3*s3 + 125*c3*c3*c4*c6*m6*s2*s5*s5 + 2000*c4*c5*c5*d5*m5*s2*s3*s3 + 2000*c3*c3*c4*d5*m5*s2*s5*s5 + 2000*c4*c5*c5*d5*m6*s2*s3*s3 + 2000*c3*c3*c4*d5*m6*s2*s5*s5 + 125*c4*c6*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m5*s2*s3*s3*s5*s5 + 125*c2*c3*c4*c5*m6*s6 + 2000*c2*c3*c4*d3*m5*s4 + 2000*c3*c3*c4*c4*c5*c5*d3*m5*s2 + 2000*c3*c3*c4*c4*d3*m5*s2*s5*s5 + 2000*c3*c3*c4*c4*d3*m6*s2*s5*s5 + 2000*c3*c3*c6*c6*d3*m6*s2*s4*s4 + 2000*c3*c3*c4*c5*c5*c6*c6*d5*m6*s2 - 2000*c2*c5*d3*m6*s3*s4*s5 + 2000*c3*c5*d5*m6*s2*s3*s5 + 2000*c3*c3*c4*c5*c5*d5*m6*s2*s6*s6 + 2000*c4*c6*c6*d5*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m6*s2*s3*s3*s5*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*d3*m5*s4 + 2000*c2*c3*c4*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*d3*m5*s4*s5*s5 - 2000*c2*c3*c4*d3*m6*s4*s5*s5 + 2000*c2*c3*c4*d3*m6*s4*s6*s6 + 2000*c3*c3*c4*c4*c5*c5*c6*c6*d3*m6*s2 + 2000*c3*c3*c4*c4*c5*c5*d3*m6*s2*s6*s6 - 2000*c2*c3*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c3*c5*c5*d5*m6*s4*s6*s6 + 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5 + 2000*c2*c5*c6*c6*d3*m6*s3*s4*s5 - 2000*c3*c5*c6*c6*d5*m6*s2*s3*s5 + 2000*c2*c5*d3*m6*s3*s4*s5*s6*s6 - 2000*c3*c5*d5*m6*s2*s3*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*c5*c5*d3*m6*s4*s6*s6 + 4000*c3*c4*c5*d3*m6*s2*s3*s5 - 2000*c2*c4*c5*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*c6*c6*d3*m6*s2*s3*s5 + 2000*c2*c4*c5*c6*c6*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*d3*m6*s2*s3*s5*s6*s6 + 2000*c2*c4*c5*d5*m6*s3*s4*s5*s6*s6 - 2000*c3*c4*c4*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5*s6*s6))/2000.0;
      */

      tau=-(g*(140*m2*s2 - 140*c3*c3*m3*s2 - 140*m3*s2*s3*s3 - 120*c2*m3*s3 - 160*c2*c3*m4*s4 + 120*c2*c4*c4*m4*s3 + 160*c3*c3*c4*m4*s2 + 2000*c3*c3*d3*m3*s2 + 120*c2*m4*s3*s4*s4 + 160*c4*m4*s2*s3*s3 + 2000*d3*m3*s2*s3*s3 + 2000*d3*m4*s2*s3*s3 - 152*c3*c3*c4*c5*c5*m5*s2 + 2000*c3*c3*c4*c4*d3*m4*s2 - 152*c4*c5*c5*m5*s2*s3*s3 - 152*c3*c3*c4*m5*s2*s5*s5 + 2000*c3*c3*d3*m4*s2*s4*s4 + 2000*c3*c3*d3*m5*s2*s4*s4 + 2000*c5*c5*d3*m5*s2*s3*s3 + 2000*c5*c5*d3*m6*s2*s3*s3 - 152*c4*m5*s2*s3*s3*s5*s5 + 2000*d3*m5*s2*s3*s3*s5*s5 + 152*c2*c3*c5*c5*m5*s4 + 152*c2*c3*m5*s4*s5*s5 + 2000*c3*c3*d3*m6*s2*s4*s4*s6*s6 + 2000*c6*c6*d3*m6*s2*s3*s3*s5*s5 + 2000*d3*m6*s2*s3*s3*s5*s5*s6*s6 - 125*c2*c3*c5*c5*c6*m6*s4 - 2000*c2*c3*c5*c5*d5*m5*s4 - 125*c2*c3*c6*m6*s4*s5*s5 - 2000*c2*c3*d5*m5*s4*s5*s5 - 2000*c2*c3*d5*m6*s4*s5*s5 - 125*c2*c4*c4*m6*s3*s5*s6 + 125*c3*c3*c5*m6*s2*s4*s6 - 125*c2*m6*s3*s4*s4*s5*s6 + 125*c5*m6*s2*s3*s3*s4*s6 + 125*c3*c3*c4*c5*c5*c6*m6*s2 + 2000*c3*c3*c4*c5*c5*d5*m5*s2 + 125*c4*c5*c5*c6*m6*s2*s3*s3 + 125*c3*c3*c4*c6*m6*s2*s5*s5 + 2000*c4*c5*c5*d5*m5*s2*s3*s3 + 2000*c3*c3*c4*d5*m5*s2*s5*s5 + 2000*c4*c5*c5*d5*m6*s2*s3*s3 + 2000*c3*c3*c4*d5*m6*s2*s5*s5 + 125*c4*c6*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m5*s2*s3*s3*s5*s5 + 125*c2*c3*c4*c5*m6*s6 + 2000*c2*c3*c4*d3*m5*s4 + 2000*c3*c3*c4*c4*c5*c5*d3*m5*s2 + 2000*c3*c3*c4*c4*d3*m5*s2*s5*s5 + 2000*c3*c3*c4*c4*d3*m6*s2*s5*s5 + 2000*c3*c3*c6*c6*d3*m6*s2*s4*s4 + 2000*c3*c3*c4*c5*c5*c6*c6*d5*m6*s2 - 2000*c2*c5*d3*m6*s3*s4*s5 + 2000*c3*c5*d5*m6*s2*s3*s5 + 2000*c3*c3*c4*c5*c5*d5*m6*s2*s6*s6 + 2000*c4*c6*c6*d5*m6*s2*s3*s3*s5*s5 + 2000*c4*d5*m6*s2*s3*s3*s5*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*d3*m5*s4 + 2000*c2*c3*c4*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*d3*m5*s4*s5*s5 - 2000*c2*c3*c4*d3*m6*s4*s5*s5 + 2000*c2*c3*c4*d3*m6*s4*s6*s6 + 2000*c3*c3*c4*c4*c5*c5*c6*c6*d3*m6*s2 + 2000*c3*c3*c4*c4*c5*c5*d3*m6*s2*s6*s6 - 2000*c2*c3*c5*c5*c6*c6*d5*m6*s4 - 2000*c2*c3*c5*c5*d5*m6*s4*s6*s6 + 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5 + 2000*c2*c5*c6*c6*d3*m6*s3*s4*s5 - 2000*c3*c5*c6*c6*d5*m6*s2*s3*s5 + 2000*c2*c5*d3*m6*s3*s4*s5*s6*s6 - 2000*c3*c5*d5*m6*s2*s3*s5*s6*s6 - 2000*c2*c3*c4*c5*c5*c6*c6*d3*m6*s4 - 2000*c2*c3*c4*c5*c5*d3*m6*s4*s6*s6 + 4000*c3*c4*c5*d3*m6*s2*s3*s5 - 2000*c2*c4*c5*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*c6*c6*d3*m6*s2*s3*s5 + 2000*c2*c4*c5*c6*c6*d5*m6*s3*s4*s5 - 4000*c3*c4*c5*d3*m6*s2*s3*s5*s6*s6 + 2000*c2*c4*c5*d5*m6*s3*s4*s5*s6*s6 - 2000*c3*c4*c4*c5*c6*c6*d5*m6*s2*s3*s5 - 2000*c3*c4*c4*c5*d5*m6*s2*s3*s5*s6*s6))/2000.0;

#endif
      return tau;
    }

    double gravity_compensation_joint_1(){


      double g=gravity_constant;

      double m1 = links_[0]->GetInertial()->Mass(); // because 0-th link in links_ is in fact 1-th link of the lwr4+ manipulator
      double m2 = links_[1]->GetInertial()->Mass(); // because 1-th link in links_ is in fact 2-th link of the lwr4+ manipulator
      double m3 = links_[2]->GetInertial()->Mass(); // because 2-th link in links_ is in fact 3-th link of the lwr4+ manipulator
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double m7 = links_[6]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"M1="<<m1<<std::endl;
        std::cout<<"M2="<<m2<<std::endl;
        std::cout<<"M3="<<m3<<std::endl;
        std::cout<<"M4="<<m4<<std::endl;
        std::cout<<"M5="<<m5<<std::endl;
        std::cout<<"M6="<<m6<<std::endl;
        std::cout<<"M7="<<m7<<std::endl;
      #endif

      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3, l2=L2, l1=L1;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);

      double tau=0;
#ifdef OPTION_1
      // wersja nr 1

      /*
      tau=-g*(c3*c4*c4*d3*m4*s2*s2*s3 - c3*d3*m4*s2*s2*s3 - c3*c5*c5*d3*m5*s2*s2*s3 - c3*c5*c5*d3*m6*s2*s2*s3 - c2*c2*c5*d5*m6*s4*s4*s5 + c3*d3*m4*s2*s2*s3*s4*s4 + c3*d3*m5*s2*s2*s3*s4*s4 - c3*d3*m5*s2*s2*s3*s5*s5 + c5*d5*m6*s2*s2*s3*s3*s5 + c3*c4*d5*m6*s2*s2*s3*s5*s5 + c4*c5*d3*m6*s2*s2*s3*s3*s5 + c2*c4*d3*m5*s2*s3*s4 + c3*c4*c4*c5*c5*d3*m5*s2*s2*s3 - c3*c3*c4*c4*c5*d5*m6*s2*s2*s5 + c2*c2*c5*c6*c6*d5*m6*s4*s4*s5 + c3*c4*c4*d3*m5*s2*s2*s3*s5*s5 + c3*c4*c4*d3*m6*s2*s2*s3*s5*s5 + c3*c6*c6*d3*m6*s2*s2*s3*s4*s4 - c3*c6*c6*d3*m6*s2*s2*s3*s5*s5 - c5*c6*c6*d5*m6*s2*s2*s3*s3*s5 + c2*c2*c5*d5*m6*s4*s4*s5*s6*s6 + c3*d3*m6*s2*s2*s3*s4*s4*s6*s6 - c3*d3*m6*s2*s2*s3*s5*s5*s6*s6 - c5*d5*m6*s2*s2*s3*s3*s5*s6*s6 + c2*c5*c5*d5*m6*s2*s3*s4 - c2*d5*m6*s2*s3*s4*s5*s5 - c3*c4*c5*c5*d5*m6*s2*s2*s3 - c3*c3*c4*c5*d3*m6*s2*s2*s5 - c2*c4*c5*c5*d3*m5*s2*s3*s4 + c2*c4*c6*c6*d3*m6*s2*s3*s4 + c3*c4*c4*c5*c5*c6*c6*d3*m6*s2*s2*s3 + c3*c3*c4*c4*c5*c6*c6*d5*m6*s2*s2*s5 - c2*c4*d3*m5*s2*s3*s4*s5*s5 - c2*c4*d3*m6*s2*s3*s4*s5*s5 + c2*c4*d3*m6*s2*s3*s4*s6*s6 + c3*c4*c4*c5*c5*d3*m6*s2*s2*s3*s6*s6 + c3*c3*c4*c4*c5*d5*m6*s2*s2*s5*s6*s6 - c2*c5*c5*c6*c6*d5*m6*s2*s3*s4 - c2*c5*c5*d5*m6*s2*s3*s4*s6*s6 + c2*c6*c6*d5*m6*s2*s3*s4*s5*s5 + c2*d5*m6*s2*s3*s4*s5*s5*s6*s6 + c2*c3*c5*d3*m6*s2*s4*s5 + c3*c4*c5*c5*c6*c6*d5*m6*s2*s2*s3 + c3*c3*c4*c5*c6*c6*d3*m6*s2*s2*s5 + c3*c4*c5*c5*d5*m6*s2*s2*s3*s6*s6 - c3*c4*c6*c6*d5*m6*s2*s2*s3*s5*s5 - c4*c5*c6*c6*d3*m6*s2*s2*s3*s3*s5 + c3*c3*c4*c5*d3*m6*s2*s2*s5*s6*s6 - c3*c4*d5*m6*s2*s2*s3*s5*s5*s6*s6 - c4*c5*d3*m6*s2*s2*s3*s3*s5*s6*s6 + 2*c2*c3*c4*c5*d5*m6*s2*s4*s5 - c2*c3*c5*c6*c6*d3*m6*s2*s4*s5 - c2*c3*c5*d3*m6*s2*s4*s5*s6*s6 - c2*c4*c5*c5*c6*c6*d3*m6*s2*s3*s4 - c2*c4*c5*c5*d3*m6*s2*s3*s4*s6*s6 - 2*c2*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2*c2*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6);
      */
      tau=-g*(c3*c4*c4*d3*m4*s2*s2*s3 - c3*d3*m4*s2*s2*s3 - c3*c5*c5*d3*m5*s2*s2*s3 - c3*c5*c5*d3*m6*s2*s2*s3 - c2*c2*c5*d5*m6*s4*s4*s5 + c3*d3*m4*s2*s2*s3*s4*s4 + c3*d3*m5*s2*s2*s3*s4*s4 - c3*d3*m5*s2*s2*s3*s5*s5 + c5*d5*m6*s2*s2*s3*s3*s5 + c3*c4*d5*m6*s2*s2*s3*s5*s5 + c4*c5*d3*m6*s2*s2*s3*s3*s5 + c2*c4*d3*m5*s2*s3*s4 + c3*c4*c4*c5*c5*d3*m5*s2*s2*s3 - c3*c3*c4*c4*c5*d5*m6*s2*s2*s5 + c2*c2*c5*c6*c6*d5*m6*s4*s4*s5 + c3*c4*c4*d3*m5*s2*s2*s3*s5*s5 + c3*c4*c4*d3*m6*s2*s2*s3*s5*s5 + c3*c6*c6*d3*m6*s2*s2*s3*s4*s4 - c3*c6*c6*d3*m6*s2*s2*s3*s5*s5 - c5*c6*c6*d5*m6*s2*s2*s3*s3*s5 + c2*c2*c5*d5*m6*s4*s4*s5*s6*s6 + c3*d3*m6*s2*s2*s3*s4*s4*s6*s6 - c3*d3*m6*s2*s2*s3*s5*s5*s6*s6 - c5*d5*m6*s2*s2*s3*s3*s5*s6*s6 + c2*c5*c5*d5*m6*s2*s3*s4 - c2*d5*m6*s2*s3*s4*s5*s5 - c3*c4*c5*c5*d5*m6*s2*s2*s3 - c3*c3*c4*c5*d3*m6*s2*s2*s5 - c2*c4*c5*c5*d3*m5*s2*s3*s4 + c2*c4*c6*c6*d3*m6*s2*s3*s4 + c3*c4*c4*c5*c5*c6*c6*d3*m6*s2*s2*s3 + c3*c3*c4*c4*c5*c6*c6*d5*m6*s2*s2*s5 - c2*c4*d3*m5*s2*s3*s4*s5*s5 - c2*c4*d3*m6*s2*s3*s4*s5*s5 + c2*c4*d3*m6*s2*s3*s4*s6*s6 + c3*c4*c4*c5*c5*d3*m6*s2*s2*s3*s6*s6 + c3*c3*c4*c4*c5*d5*m6*s2*s2*s5*s6*s6 - c2*c5*c5*c6*c6*d5*m6*s2*s3*s4 - c2*c5*c5*d5*m6*s2*s3*s4*s6*s6 + c2*c6*c6*d5*m6*s2*s3*s4*s5*s5 + c2*d5*m6*s2*s3*s4*s5*s5*s6*s6 + c2*c3*c5*d3*m6*s2*s4*s5 + c3*c4*c5*c5*c6*c6*d5*m6*s2*s2*s3 + c3*c3*c4*c5*c6*c6*d3*m6*s2*s2*s5 + c3*c4*c5*c5*d5*m6*s2*s2*s3*s6*s6 - c3*c4*c6*c6*d5*m6*s2*s2*s3*s5*s5 - c4*c5*c6*c6*d3*m6*s2*s2*s3*s3*s5 + c3*c3*c4*c5*d3*m6*s2*s2*s5*s6*s6 - c3*c4*d5*m6*s2*s2*s3*s5*s5*s6*s6 - c4*c5*d3*m6*s2*s2*s3*s3*s5*s6*s6 + 2*c2*c3*c4*c5*d5*m6*s2*s4*s5 - c2*c3*c5*c6*c6*d3*m6*s2*s4*s5 - c2*c3*c5*d3*m6*s2*s4*s5*s6*s6 - c2*c4*c5*c5*c6*c6*d3*m6*s2*s3*s4 - c2*c4*c5*c5*d3*m6*s2*s3*s4*s6*s6 - 2*c2*c3*c4*c5*c6*c6*d5*m6*s2*s4*s5 - 2*c2*c3*c4*c5*d5*m6*s2*s4*s5*s6*s6);

#endif
      return tau;
    }

    /* Gravity compensation - algorithm nr 2 */
    void getGravComp2(std::array<double, 7> &t) {
      //
      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Gravitation compensation based on Newton-Euler dynamics equations"<<std::endl;
      #endif
      //
      // t[6]=gravity_compensation_joint_7();
      // t[5]=gravity_compensation_joint_6(); //t[5]=0;
      // t[4]=gravity_compensation_joint_5(); //t[4]=0;
      // t[3]=gravity_compensation_joint_4(); //t[3]*=1.2;
      // t[2]=gravity_compensation_joint_3(); t[2]=0;
      // t[1]=gravity_compensation_joint_2(); //t[1]=t[1]*1.2;
      // t[0]=gravity_compensation_joint_1(); //t[0]=0;

// eksperyment
      t[6]=gravity_compensation_joint_7();
      t[5]=gravity_compensation_joint_6(); //t[5]=0;
      t[4]=gravity_compensation_joint_5(); //t[4]=0;
      t[3]=gravity_compensation_joint_4(); //t[3]*=1.2;
      t[2]=gravity_compensation_joint_3(); //           t[2]=0;
      t[1]=gravity_compensation_joint_2(); //t[1]=t[1]*1.2;
      t[0]=gravity_compensation_joint_1(); //t[0]=0;


      // change the sign of the torque
      #ifdef PRINT_DEBUG_INFO
        std::cout<<std::endl;
        for (int i = 0; i < 7; ++i) {
            std::cout<<"[Optional] t["<<i<<"]="<<t[i]<<std::endl;
        }
        std::cout<<std::endl;
      #endif
    }

    /*
      gravity compensation - basic algorithm
    */
    void getGravComp(std::array<double, 7 > &t) {
        // gravity vector
        ignition::math::Vector3d gr = gazebo::physics::get_world()->Gravity();
        ignition::math::Vector3d tool_com;
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"tool_com="<<tool_com<<std::endl;
        #endif

        // tool mass
        double tool_mass = 0;

        // pointer to the last link from the links_ vector - i.e. end effector
        gazebo::physics::LinkPtr link = links_[6];
        // pointer to the last joint from the joints_ vector
        gazebo::physics::JointPtr joint = joints_[6];
        // get the world pose of the link
        ignition::math::Pose3d T_W_L7 = link->WorldPose(); // get the global position of the last link, i.e. in simulation the last link is 6-th link
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"T_W_L7="<<T_W_L7<<std::endl;
        #endif
        // add to T_W_L7 pose vector tool_com (is it empty?)
        ignition::math::Vector3d cog = T_W_L7.CoordPositionAdd( tool_com ); // calculate the center of gravity of link nr 6 (in fact we number all link from 0 to 6, so in fact here the last link is 6-th but in reality we say it is the 7-th link)
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"tool_com for link 6 => tool_com="<<tool_com<<std::endl;
          std::cout<<"cog for link 6 => cog="<<cog<<std::endl;
        #endif
        // calculate new vector which is vector cog - position of last joint
        ignition::math::Vector3d r = cog - joint->WorldPose().Pos(); // calculate the distance between the global joint position and the center of gravity (i.e. it's a center of mass)
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"r for link 6 => r="<<r<<std::endl;
        #endif

        // set a mass to tool_mass - i.e. it equals zero
        double mass = tool_mass; // tool mass we assume equals zero
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"mass="<<mass<<std::endl;
        #endif
        // calculate torque as a cross product of two vectors r and gravity vector multiplied by mass (which is still zero)
        ignition::math::Vector3d torque = r.Cross(mass * gr); // we calculate the torque exerting on the last joint as a cross product of r and (mass * gr) [arm x mass * gravity constant], pay attention that the mass of the last link is zero
        // calculate axis
        ignition::math::Vector3d axis = joint->GlobalAxis(0); // rotation axis of joint nr 6 in global position
        t[6] = axis.Dot(torque); // dot product of axis and torque is a torque compansating the gravity for the last link

        #ifdef PRINT_DEBUG_INFO
          std::cout<<"#####################################"<<std::endl;
          std::cout<<"Joint position for i=6-th joint: "<<joint->WorldPose().Pos()<<std::endl;
          std::cout<<"Axis for i=6-th joint: "<<axis<<std::endl;
          std::cout<<"Torque for i=6-th joint: "<<t[6]<<std::endl;
        #endif
        // for each link within links_ - except the 6-th link
        for (int i = 6; i > 0; i--) {
            link = links_[i-1]; // get the (i-1)th link
            joint = joints_[i-1]; // get the (i-1)th joint
            // WorldCoGPose - get the pose of the body's center of gravity in the world coordinate frame
            // now we calculate the center of gravity for all links already visited, i.e. (i-1) to 6, we are using weighted mean
            cog = (cog * mass + link->WorldCoGPose().Pos() * link->GetInertial()->Mass()) / (mass+link->GetInertial()->Mass()); // we calculate here the weighted mean, based on this we calculate the center of gravity of the links (from i-1-th link do 6-th link)
            #ifdef PRINT_DEBUG_INFO
              std::cout<<"cog="<<cog<<std::endl;
            #endif
            // update the total mass of already visited links, i.e. (i-1) to 6
            mass += link->GetInertial()->Mass(); // here we calculate the total sum of the manipulator (iteratively adding masses of links starting from end-effector, which has zero mass)
            #ifdef PRINT_DEBUG_INFO
              std::cout<<"mass["<<i-1<<"]="<<mass<<std::endl;
            #endif
            // caluclate the distance between the joint position and the center of gravity of all already visited joints
            r = cog - joint->WorldPose().Pos();
            // calculate the torque excerting on joint, as a cross product of arm and the gravitation force acting on the arm
            torque = r.Cross(mass * gr);

            // global axis of joint (i-1) - i.e. rotation axis of joint, in other words the z-th vector from transformation matrix ^0_(i-1)T
            axis = joint->GlobalAxis(0);

            // torque exerting on joint (i-1) along, why dot product? because we calculate torque along z-th axis from transformation matrix (rotation axis of joint i-1 from poiint of view of base frame)
            t[i-1] = axis.Dot(torque);

            #ifdef PRINT_DEBUG_INFO
              std::cout<<"#####################################"<<std::endl;
              std::cout<<"Joint position for i="<<i-1<<"-th joint "<<joint->WorldPose().Pos()<<std::endl;
              std::cout<<"Axis for i="<<i-1<<"-th joint "<<axis<<std::endl;
              std::cout<<"Torque for i="<<i-1<<"-th joint "<<t[i-1]<<std::endl;
            #endif
        }
        #ifdef PRINT_DEBUG_INFO
          std::cout<<"#####################################"<<std::endl;
          std::cout<<""<<std::endl;
          std::cout<<""<<std::endl;
          std::cout<<"#####################################"<<std::endl;
        #endif
        // change the sign of the torque
        for (int i = 0; i < 7; ++i) {
            t[i] = -t[i];
            #ifdef PRINT_DEBUG_INFO
              std::cout<<"[Original] t["<<i<<"]="<<t[i]<<std::endl;
            #endif
        }
    } // end of function getGravComp



// just get center of gravity of each links
    void getCenterOfGravityJustForTests(){

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Get center of gravity of links"<<std::endl;

        for(int i=0; i<7; i++){
          std::cout<<"CoG of link "<<i+1<<"="<<links_[i]->WorldCoGPose().Pos()<<std::endl;

        }
        for(int i=0; i<7; i++){
          std::cout<<"Joint position "<<i+1<<"="<<joints_[i]->WorldPose().Pos()<<std::endl;
        }
        for(int i=0; i<7; i++){
          std::cout<<"Link position "<<i+1<<"="<<links_[i]->WorldPose().Pos()<<std::endl;
        }

        for(int i=0; i<7; i++){
          std::cout<<"Delta "<<i+1<<"="<<links_[i]->WorldPose().Pos()-links_[i]->WorldCoGPose().Pos()<<std::endl;
        }
      #endif
    }

#ifdef EXTERNAL_CALCULATIONS
    /* Set torque based on received data */
    void setTorqueBasedOnReceiveDataFromSharedMemory(){
      /* Read torques from shared memory */
      struct lwr4_joints received_torque;

      //received_torque=shm_torque_consumer->readSynchronously();
      // or
      received_torque=shm_torque_consumer->readAsynchronously();

      // get table of joints
      for(int i=0;i<7;i++){
        torque[i]=received_torque._joints[i];
        //std::cout<<"Received data - torque["<<i+1<<"]="<<received_torque._joints[i]<<std::endl;
      }
    }

    /* Send current position and velocity of the last frame, i.e. end-effector frame */
    void sendCurrentLWRManipulatorParameters(){

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"[sendCurrentLWRManipulatorParameters] -- Start "<<std::endl;
      #endif
      struct lwr4_kinematics_params current_lwr_params;

      current_lwr_params.theta1=joints_[0]->Position(0); //std::cout<<"sets theta1="<<joints_[0]->Position(0)<<std::endl;
      current_lwr_params.theta2=joints_[1]->Position(0); //std::cout<<"sets theta2="<<joints_[1]->Position(0)<<std::endl;
      current_lwr_params.theta3=joints_[2]->Position(0); //std::cout<<"sets theta3="<<joints_[2]->Position(0)<<std::endl;
      current_lwr_params.theta4=joints_[3]->Position(0); //std::cout<<"sets theta4="<<joints_[3]->Position(0)<<std::endl;
      current_lwr_params.theta5=joints_[4]->Position(0); //std::cout<<"sets theta5="<<joints_[4]->Position(0)<<std::endl;
      current_lwr_params.theta6=joints_[5]->Position(0); //std::cout<<"sets theta6="<<joints_[5]->Position(0)<<std::endl;
      current_lwr_params.theta7=joints_[6]->Position(0); //std::cout<<"sets theta7="<<joints_[6]->Position(0)<<std::endl;

      current_lwr_params.x_current=links_[6]->WorldPose().Pos().X()-links_[0]->WorldPose().Pos().X();
      current_lwr_params.y_current=links_[6]->WorldPose().Pos().Y()-links_[0]->WorldPose().Pos().Y();
      current_lwr_params.z_current=links_[6]->WorldPose().Pos().Z();//-links_[0]->WorldPose().Pos().Z();
      //########################################
      current_lwr_params.roll_current=equilibrium_roll;
      current_lwr_params.pitch_current=equilibrium_pitch;
      current_lwr_params.yaw_current=equilibrium_yaw;

      current_lwr_params.v_x=links_[6]->WorldLinearVel().X();
      current_lwr_params.v_y=links_[6]->WorldLinearVel().Y();
      current_lwr_params.v_z=links_[6]->WorldLinearVel().Z();

      current_lwr_params.w_roll=0;
      current_lwr_params.w_pitch=0;
      current_lwr_params.w_yaw=0;

      /* Writes asynchronously - or in other way, e.g. Synchornously */
      shm_parameters_producer->writeAsynchronously(current_lwr_params);
    }
#endif

    // Called by the world update start event
    public: void OnUpdate()
    {
      torque.fill(0);      // initialize all torques to 0
      #ifdef EXTERNAL_CALCULATIONS /* calculation done in external process, data sent through share memory */
        /* Shared memory - test */
        //float msg_tmp = rand() % 360;
        //std::cout<<"Writes message: "<<msg_tmp<<std::endl;
        //shm_producer->writeSynchronously(msg_tmp);

        /* Update torque from shared memory */
        setTorqueBasedOnReceiveDataFromSharedMemory();

      #endif // END of EXTERNAL_CALCULATIONS

      #ifdef GAZEBO_CALCULATIONS
      // WORKS
      //LWR4KinematicsDynamics lwr=LWR4KinematicsDynamics(0, 30, 0, 10);
      //int i = lwr.function(4);

        // get gravity compensation torques - our function
        //getGravComp(t);
        //t.fill(0);      // initialize all torques to 0

        /*
        SUCCESS !!! - my version of gravity compensation based on Euler-Newton equations !!! - it works!
        */

        // GRAVITY COMPENSATION!!!
        getGravComp2(torque); // gravity compensation version based on Euler-Newton equations (my version)

        //getCenterOfGravityJustForTests();
// ###########################
        // update equilibrium
//        equilibrium=equilibrium_global;
//        UpdateEquilibirum();
// ###########################

// ##############################################
        // impedance control in joints
        //impedanceControlInJointSpace(t);    // <<<  =================================================================
// ##############################################

// ##############################################
        // impedance control in joints
        impedanceControlInCartesianSpace(torque); // <<<  =================================================================
// ##############################################



        // just to test impedance control in cartesian space
        updateCartesianImpedance();
      #endif // END of GAZEBO_CALCULATIONS


      // ### - just for tests
      #ifdef CALCULATE_SAMPLING_PERIOD
      {
        std::cout<<"CALCULATE SAMPLING PERIOD"<<std::endl;
        std::chrono::duration<double> elapsed_time_between_iterations = std::chrono::system_clock::now()-end_time;
        std::cout<< "Elapsed between iterations: "<<elapsed_time_between_iterations.count()<<std::endl;
        end_time=std::chrono::system_clock::now();
      }
      #endif // CALCULATE_SAMPLING_PERIOD

      // apply torques
      setForces(torque);

      saveEndEffectorPoseToFile(); // save to file current position of end-effector

      #ifdef EXTERNAL_CALCULATIONS
        /* SEND current position of end-effector frame */
        sendCurrentLWRManipulatorParameters();
      #endif

    } // end of function OnUpdate

    void updateCartesianImpedance(){
      double radius=0.3;
      double origin_x=0;
      double origin_y=0;
      double origin_z=0.5;

      // ############# JUST TO print time - i.e. calculate the sampling period ########################



      _iterations2++; // how many iterations are within a single cycle (waiting a whole cycle to change the desired cartesian position)
      if(_iterations2>ITERATIONS_IN_ONE_CYCLE){
        _iterations2=0; // if number of iterations is greater then ITERATIONS_IN_ONE_CYCLE (the waiting time has been ended, set zero and calculate new cartesian position for end-effector)
      }
      else{
        return;
      }
      //saveEndEffectorPoseToFile(); // save to file current position of end-effector
      if(_flag) // one direction of movement of manipulator's end-effector
        _iterations++;
      else      // another direction of movement of manipulator's end-effector
        _iterations--;

      if(_iterations>MAX_ITERATIONS){ // check whether the whole movement in a desired direction has ended
        _iterations--;
        _flag=false; // change direction of movement
      }
      if(_iterations<0){ // check whether the whole movement in a desired direction has ended
        _iterations++;
        _flag=true; // change direction of movement
      }

      // ###########################################################################################################################
      // ######## Calculate desired position in cartesian space, i.e. equilibrium point #############################
      // ###########################################################################################################################

      // in X-Y plane
      // double theta=(_iterations/MAX_ITERATIONS) * 360 * 3.14/180.0;
      // std::cout<<"!!!!!ITERATIONS="<<_iterations<<" theta="<<theta<<std::endl;
      // equilibrium_x= origin_x + radius * cos(theta);
      // equilibrium_y= origin_y + radius * sin(theta);

      // in Z-Y plane
      double theta=(_iterations/MAX_ITERATIONS) * 360 * 3.14/180.0;
      #ifdef PRINT_DEBUG_INFO
        std::cout<<"!!!!!ITERATIONS="<<_iterations<<" theta="<<theta<<" EQUILIBRIUM=("<<equilibrium_x<<","<<equilibrium_y<<","<<equilibrium_z<<")"<<std::endl;
      #endif
      equilibrium_z= origin_z + radius * sin(theta);
      equilibrium_y= origin_y + radius * cos(theta);

      // equilibrium in X coordinate is always the same - i.e. as defined at the beginning of this file: equilibrium_x=0.56;
      // ###########################################################################################################################
      // ###########################################################################################################################


      // ############# JUST TO print time - i.e. calculate the sampling period ########################
      #ifdef CALCULATE_SAMPLING_PERIOD
      {
        end_time=std::chrono::system_clock::now();
        std::cout<<"CALCULATE SAMPLING PERIOD"<<std::endl;
        std::chrono::duration<double> elapsed_seconds = end_time-start_time;
        std::cout<< "Elapsed time for a single iteration, i.e. time between calculating a new cartesian position: "<<elapsed_seconds.count()<<std::endl;
        start_time=end_time;
      }
      #endif // CALCULATE_SAMPLING_PERIOD

      // #########################################################


    }

// impedance control in joints
    void impedanceControlInJointSpace(std::array<double, 7> &t) {

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Impedance Control In Joint Space"<<std::endl;
      #endif
      // declare equilibrium point - set the desired position of the kinematic chain
      //std::array<double, 7 > eq({0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}); // almost vertical
      //std::array<double, 7 > eq ({0.04, 0, 0, 0, 0, 0, 0}); // joint angles in radians
      std::array<double, 7> eq = equilibrium;

      // calculate spring forces - becasue we utilise the impedance control - in joint space!
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      double k = 10;    // stiffness constant
      for (int i = 0; i < t.size(); ++i) {
          double diff = eq[i] - joints_[i]->Position(0); // the difference between the equilibrium point and the current joint positions (Position(0) returns the current position of the axis nr 0)
          t[i] += k * diff; // add to torque additional force ?
      }
    }
// # end of impedanceControlInJointSpace


    void saveInFile(std::string fileName, std::string what){
      std::ofstream myfile;
      myfile.open (fileName, std::ios::app);
      myfile << what;
      myfile.close();
    }

    void saveBasePoseToFile(){

      std::ostringstream strs;
      strs << links_[0]->WorldPose().Pos().X()<<" "<<links_[0]->WorldPose().Pos().Y()<<" "<<0<<" ";
      strs<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<"\n";
      std::string what = strs.str();
      saveInFile("pozycje.txt", what);
    }

    void saveEndEffectorPoseToFile(){

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      std::ostringstream strs;
      strs << links_[6]->WorldPose().Pos().X()<<" "<<links_[6]->WorldPose().Pos().Y()<<" "<<links_[6]->WorldPose().Pos().Z()<<" ";
      strs<<theta1<<" "<<theta2<<" "<<theta3<<" "<<theta4<<" "<<theta5<<" "<<theta6<<" "<<theta7<<"\n";
      std::string what = strs.str();
      saveInFile("pozycje.txt", what);
    }
// impedance control in joints
    void impedanceControlInCartesianSpace(std::array<double, 7> &t) {
      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Impedance Control In Cartesian Space"<<std::endl;
      #endif
      //ignition::math::Pose3d pos=ignition::math::Pose3d(links_[6]->WorldPose().Pos().X(), links_[6]->WorldPose().Pos().Y(), links_[6]->WorldPose().Pos().Z());


      //_end_effector_position_vector.push_back(pos);

// ##################################################################################################
      double d7= D7, d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3, l2=L2, l1=L1;

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);
      double theta7=joints_[6]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6), c7=cos(theta7);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6), s7=sin(theta7);
// ##################################################################################################

// ##################################################################################################
      // declare equilibrium point - set the desired position of the kinematic chain
      // equilibrium point (x,y,z)
      double x_desired= equilibrium_x;
      double y_desired= equilibrium_y;
      double z_desired= equilibrium_z;
      double roll_desired = equilibrium_roll;
      double pitch_desired = equilibrium_pitch;
      double yaw_desired = equilibrium_yaw;
// ##################################################################################################



// ##################################################################################################
      // calculate spring forces - becasue we utilise the impedance control - in cartesian space!
// ##################################################################################################

// ##################################################################################################
      // double x_current=links_[6]->WorldPose().Pos().X()-links_[0]->WorldPose().Pos().X();
      // double y_current=links_[6]->WorldPose().Pos().Y()-links_[0]->WorldPose().Pos().Y();
      // double z_current=links_[6]->WorldPose().Pos().Z();
      // double roll_current=links_[6]->WorldPose().Rot().Yaw();
      // double pitch_current=links_[6]->WorldPose().Rot().Pitch();
      // double yaw_current=links_[6]->WorldPose().Rot().Roll();

      // ###################################################################################
      // in XY AXIS
      // double x_current=links_[6]->WorldPose().Pos().X()-links_[0]->WorldPose().Pos().X();
      // double y_current=links_[6]->WorldPose().Pos().Y()-links_[0]->WorldPose().Pos().Y();
      // double z_current=z_desired;//links_[6]->WorldPose().Pos().Z()-links_[0]->WorldPose().Pos().Z();
      //########################################

      // ###################################################################################
      // in XZ AXIS
      double x_current=links_[6]->WorldPose().Pos().X()-links_[0]->WorldPose().Pos().X();
      double y_current=links_[6]->WorldPose().Pos().Y()-links_[0]->WorldPose().Pos().Y();
      double z_current=links_[6]->WorldPose().Pos().Z();//-links_[0]->WorldPose().Pos().Z();
      //########################################
      double roll_current=roll_desired;
      double pitch_current=pitch_desired;
      double yaw_current=yaw_desired;
// ##################################################################################################

// ##################################################################################################
      // linear and angular velocities of the end-effector of LWR4+ manipulator
      // double v_x=links_[6]->WorldLinearVel().X();
      // double v_y=links_[6]->WorldLinearVel().Y();
      // double v_z=links_[6]->WorldLinearVel().Z();
      //double w_roll=links_[6]->WorldAngularVel().X();
      //double w_pitch=links_[6]->WorldAngularVel().Y();
      //double w_yaw=links_[6]->WorldAngularVel().Z();

// #################    in XY AXIS
      // double v_x=links_[6]->WorldLinearVel().X();
      // double v_y=links_[6]->WorldLinearVel().Y();
      // double v_z=0;//links_[6]->WorldLinearVel().Z();

// #################    in ZY AXIS
      double v_x=links_[6]->WorldLinearVel().X();
      double v_y=links_[6]->WorldLinearVel().Y();
      double v_z=links_[6]->WorldLinearVel().Z();

      double w_roll=0;
      double w_pitch=0;
      double w_yaw=0;
// ##################################################################################################
      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Current position of end-effector (X,Y,Z)=("<<x_current<<","<<y_current<<","<<z_current<<") angles (ROLL,PITCH,YAW)="<<roll_current<<","<<pitch_current<<","<<yaw_current<<")"<<std::endl;
      #endif

      // difference_k means the distance between k_desired and k_current along k-axis
      //double difference_x=x_desired-x_current;
      //double difference_y=y_desired-y_current;
      //double difference_z=z_desired-z_current;
      //double difference_roll=roll_desired-roll_current;
      //double difference_pitch=pitch_desired-pitch_current;
      //double difference_yaw=yaw_desired-yaw_current;
      double difference_x=x_desired-x_current;
      double difference_y=y_desired-y_current;
      double difference_z=z_desired-z_current;
      double difference_roll=0;
      double difference_pitch=0;
      double difference_yaw=0;

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Difference between desired and current position of end-effector (X,Y,Z)=("<<difference_x<<","<<difference_y<<","<<difference_z<<") angles (ROLL,PITCH,YAW)="<<roll_current<<","<<pitch_current<<","<<yaw_current<<")"<<std::endl;
      #endif
      // delta time - time between two iterations
      //double delta_t =0.0001;

      // stiffness constant
      double k=0;
      // stiffness matrix components
      double k11=k, k12=k, k13=k, k14=k, k15=k, k16=k, k21=k, k22=k, k23=k, k24=k, k25=k, k26=k, k31=k, k32=k, k33=k, k34=k, k35=k, k36=k, k41=k, k42=k, k43=k, k44=k, k45=k, k46=k, k51=k, k52=k, k53=k, k54=k, k55=k, k56=k, k61=k, k62=k, k63=k, k64=k, k65=k, k66=k;

      double k_diag=30; // 40 ok
      // set up diagonal parameters of stiffness matrix
      k11=k_diag; k22=k_diag; k33=k_diag; k44=k_diag; k55=k_diag; k66=k_diag;

      //damping constant
      double d=0;
      // damping matrix components
      double d11=d, d12=d, d13=d, d14=d, d15=d, d16=d, d21=d, d22=d, d23=d, d24=d, d25=d, d26=d, d31=d, d32=d, d33=d, d34=d, d35=d, d36=d, d41=d, d42=d, d43=d, d44=d, d45=d, d46=d, d51=d, d52=d, d53=d, d54=d, d55=d, d56=d, d61=d, d62=d, d63=d, d64=d, d65=d, d66=d;
      double d_diag=10;
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

      #ifdef PRINT_DEBUG_INFO
        std::cout<<"Calculated torques - impedance control in Cartesian space: "<<std::endl;
        std::cout<<"[Torque] t1 = "<<t1<<std::endl;
        std::cout<<"[Torque] t2 = "<<t2<<std::endl;
        std::cout<<"[Torque] t3 = "<<t3<<std::endl;
        std::cout<<"[Torque] t4 = "<<t4<<std::endl;
        std::cout<<"[Torque] t5 = "<<t5<<std::endl;
        std::cout<<"[Torque] t6 = "<<t6<<std::endl;
        std::cout<<"[Torque] t7 = "<<t7<<std::endl;
      #endif

      t[0]+=t1;
      t[1]+=t2;
      t[2]+=t3;
      t[3]+=t4;
      t[4]+=t5;
      t[5]+=t6;
      t[6]+=t7;

      // for (int i = 0; i < t.size(); ++i) {
      //     double diff = eq[i] - joints_[i]->Position(0); // the difference between the equilibrium point and the current joint positions (Position(0) returns the current position of the axis nr 0)
      //     t[i] += k * diff; // add to torque additional force ?
      // }
    }
// # end of impedanceControlInCartesianSpace


// #################################
    // update equilibrium
    public: void UpdateEquilibirum(){

      equilibrium[0]+=eq_0_step;
      equilibrium[3]+=eq_3_step;
      if(equilibrium[0]>3.14) {
        equilibrium[0]=3.14;
        eq_0_step*=-1;
      }
      else if(equilibrium[0]<-3.14){
        equilibrium[0]=-3.14;
        eq_0_step*=-1;
      }

      if(equilibrium[3]>1.14) {
        equilibrium[3]=1.14;
        eq_3_step*=-1;
      }
      else if(equilibrium[3]<-1.14){
        equilibrium[3]=-1.14;
        eq_3_step*=-1;
      }


    }

    private: void subscribe_callback_function(AnyPtr & _msg){
      int i;
      #ifdef PRINT_DEBUG_INFO
        std::cout << "Message received:\nMessage type="<<_msg->type()<<std::endl;
      #endif
      if(_msg->type()==2){ // double -> angle
        #ifdef PRINT_DEBUG_INFO
          std::cout << "Double="<<_msg->double_value()<<std::endl;
        #endif
//        equilibrium_global[0]=_msg->double_value();
        equilibrium[kinematic_chain_index]=_msg->double_value();
      }
      else if(_msg->type()==3){ // int -> index of kinematic chain
 	      i=_msg->int_value();
        #ifdef PRINT_DEBUG_INFO
          std::cout << "Int="<<i<<std::endl;
        #endif
	      if(i>=0 && i<=6){
          kinematic_chain_index=i;
	       }
      }
    }

    private: void subscribe_callback_function_kuka_joints(KukaJointsPtr & _msg){
      int i;
      equilibrium[0]=_msg->joint_0();
      equilibrium[1]=_msg->joint_1();
      equilibrium[2]=_msg->joint_2();
      equilibrium[3]=_msg->joint_3();
      equilibrium[4]=_msg->joint_4();
      equilibrium[5]=_msg->joint_5();
      equilibrium[6]=_msg->joint_6();

      #ifdef PRINT_DEBUG_INFO
        std::cout << "Message received:\n\
	       Joint_0="<<_msg->joint_0()<<
	        "\nJoint_1="<<_msg->joint_1()<<
	         "\nJoint_2="<<_msg->joint_2()<<
	          "\nJoint_3="<<_msg->joint_3()<<
	           "\nJoint_4="<<_msg->joint_4()<<
	            "\nJoint_5="<<_msg->joint_5()<<
	             "\nJoint_6="<<_msg->joint_6()<<std::endl;
      #endif
    }

    private: double eq_0_step, eq_3_step;
    public: std::array<double,7> equilibrium;
    public: double equilibrium_x;
    public: double equilibrium_y;
    public: double equilibrium_z;
    public: double equilibrium_roll;
    public: double equilibrium_pitch;
    public: double equilibrium_yaw;

    public: double x_last;
    public: double y_last;
    public: double z_last;
    public: double roll_last;
    public: double pitch_last;
    public: double yaw_last;

    public: double _iterations; // for impedance control test
    public: double _iterations2; // for impedance control test
    public: double _flag; // for impedance control test

    public: std::vector<ignition::math::Pose3d> _end_effector_position_vector;

    private: int kinematic_chain_index;
// #################################


    // Pointer to the model
    private: physics::ModelPtr model_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Pointer to the subscriber
    private: transport::SubscriberPtr sub, sub_kuka_joints;

    private:
    // vector of joint pointers
    std::vector<gazebo::physics::JointPtr > joints_;
    // vector of link pointers
    std::vector<gazebo::physics::LinkPtr > links_;


    /* For communication - shared memory */
    //SharedMemory<float> *shm_producer;
    /* Shared memory - torque calculated for gazebo simulation */
    SharedMemory<struct lwr4_joints> *shm_torque_consumer;

    /* Shared memory - manipulator parameters, such as angles in joint space (theta) and velocities */
    SharedMemory<struct lwr4_kinematics_params> *shm_parameters_producer;

    /* Manipulator torque */
    std::array<double, 7 > torque;

    /* Measuring the iteration time */
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> end_time;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelKukaLwr)
}
