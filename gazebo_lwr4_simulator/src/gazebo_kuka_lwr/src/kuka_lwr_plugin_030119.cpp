/*
 Copyright (c) 2018, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

 Author: Dawid Seredynski
*/

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
        std::cout << "plugin loaded" << std::endl;

// ####################################################
    // test
    // set equilibrium to base position
    equilibrium= {0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
    eq_0_step=0.001;
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


// ############################################

    } // end of function Load

    // set forces to joints based on given torques
    void setForces(const std::array<double, 7 > &t) {
        for (int i=0; i<joints_.size(); i++) {
            joints_[i]->SetForce(0, t[i]); // axis 0 jest default
        }
    } // end of function setForces

    // gravity compensation
    void getGravComp(std::array<double, 7 > &t) {
        // gravity vector
        ignition::math::Vector3d gr = gazebo::physics::get_world()->Gravity();

        ignition::math::Vector3d tool_com;

        // tool mass
        double tool_mass = 0;

        // pointer to the last link from the links_ vector - i.e. end effector
        gazebo::physics::LinkPtr link = links_[6];
        // pointer to the last joint from the joints_ vector
        gazebo::physics::JointPtr joint = joints_[6];
        // get the world pose of the link
        ignition::math::Pose3d T_W_L7 = link->WorldPose();

        // add to T_W_L7 pose vector tool_com (is it empty?)
        ignition::math::Vector3d cog = T_W_L7.CoordPositionAdd( tool_com );
        // calculate new vector which is vector cog - position of last joint
        ignition::math::Vector3d r = cog - joint->WorldPose().Pos();

        // set a mass to tool_mass - i.e. it equals zero
        double mass = tool_mass;
        // calculate torque as a cross product of two vectors r and gravity vector multiplied by mass (which is still zero)
        ignition::math::Vector3d torque = r.Cross(mass * gr);
        // calculate axis
        ignition::math::Vector3d axis = joint->GlobalAxis(0);
        // dot product of axis and torque is a torque compansating the gravity for the last link
        t[6] = axis.Dot(torque);

        // for each link within links_
        for (int i = 6; i > 0; i--) {
            link = links_[i-1];
            joint = joints_[i-1];
            // WorldCoGPose - get the pose of the body's center of gravity in the world coordinate frame
            cog = (cog * mass + link->WorldCoGPose().Pos() * link->GetInertial()->Mass()) / (mass+link->GetInertial()->Mass());
            mass += link->GetInertial()->Mass();
            r = cog - joint->WorldPose().Pos();
            torque = r.Cross(mass * gr);
            axis = joint->GlobalAxis(0);
            t[i-1] = axis.Dot(torque);
        }

        // change the sign of the torque
        for (int i = 0; i < 7; ++i) {
            t[i] = -t[i];
        }
    } // end of function getGravComp

    // Called by the world update start event
    public: void OnUpdate()
    {
        std::array<double, 7 > t;
        t.fill(0);      // initialize all torques to 0

        // get gravity compensation torques - our function
        getGravComp(t);

// ###########################
        // update equilibrium
//        equilibrium=equilibrium_global;
//        UpdateEquilibirum();

// ###########################

        // declare equilibrium point - set the desired position of the kinematic chain
        //std::array<double, 7 > eq({0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}); // almost vertical
        //std::array<double, 7 > eq ({0.04, 0, 0, 0, 0, 0, 0}); // joint angles in radians
        std::array<double, 7> eq = equilibrium;

        // calculate spring forces - becasue we utilise the impedance control
        double k = 10.0;    // stiffness constant
        for (int i = 0; i < t.size(); ++i) {
            double diff = eq[i] - joints_[i]->Position(0); // the difference between the equilibrium point and the current joint positions (Position(0) returns the current position of the axis nr 0)
            t[i] += k * diff; // add to torque additional force ?
        }

        // apply torques
        setForces(t);

    } // end of function OnUpdate

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
      std::cout << "Message received:\nMessage type="<<_msg->type()<<std::endl;
      if(_msg->type()==2){ // double -> angle
        std::cout << "Double="<<_msg->double_value()<<std::endl;
//        equilibrium_global[0]=_msg->double_value();
        equilibrium[kinematic_chain_index]=_msg->double_value();
      }
      else if(_msg->type()==3){ // int -> index of kinematic chain
 	i=_msg->int_value();
        std::cout << "Int="<<i<<std::endl;
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
      std::cout << "Message received:\n\
	Joint_0="<<_msg->joint_0()<<
	"\nJoint_1="<<_msg->joint_1()<<
	"\nJoint_2="<<_msg->joint_2()<<
	"\nJoint_3="<<_msg->joint_3()<<
	"\nJoint_4="<<_msg->joint_4()<<
	"\nJoint_5="<<_msg->joint_5()<<
	"\nJoint_6="<<_msg->joint_6()<<std::endl;
    }

    private: double eq_0_step, eq_3_step;
    public: std::array<double,7> equilibrium;

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



  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelKukaLwr)
}
