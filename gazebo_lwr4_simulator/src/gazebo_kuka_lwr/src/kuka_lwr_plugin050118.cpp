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

//#define gravity_constant 9.80665 // gravity acceleration - from General Conference on Weights and Measures - 1901 year
#define gravity_constant 9.81 // gravity acceleration - option 2

#define L1 0.2 // l1=0.2m
#define L2 0.2 // l2=0.2m
#define L3 0.2 // l3=0.2m
#define L4 0.195 // l4=0.195m
#define L5 0.195 // l5=0.195m

#define D1 0.31   // d1=0.31m
#define D3 0.4    // d3=0.4m
#define D5 0.39   // d5=0.39m
#define D7 0.078  // d7=0.078m

//#define OPTION_1
#define OPTION_2

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
    //equilibrium= {0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}; // equilibrium joint position
    equilibrium= {0, 0, 0, 0, 0, 0.4, 0}; // equilibrium joint position
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


// ############################################

    } // end of function Load

    // set forces to joints based on given torques
    void setForces(const std::array<double, 7 > &t) {
        for (int i=0; i<joints_.size(); i++) {
            joints_[i]->SetForce(0, t[i]); // axis 0 jest default
            std::cout<<"Torque["<<i<<"]="<<t[i]<<std::endl;
        }
    } // end of function setForces

    double gravity_compensation_joint_7(){
      double tau=0;
      return tau;
    }

    double gravity_compensation_joint_6(){

      double g=gravity_constant;

      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d5 = D5, d3=D3, d1=D1;

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);

#ifdef OPTION_1
      // wersja nr 1
      double tau=0;
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos(); // center of gravity for 6-th link
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y();
      double tau=-cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
#endif
      return tau;
    }

    /////////////////////////////////////////////

    double gravity_compensation_joint_5(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;
      double g=gravity_constant;
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d5 = D5, d3=D3, d1=D1;
      double l5= L5;

      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);

#ifdef OPTION_1
      // wersja nr 1
      double tau=-g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2);
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y(), cogP66z=cogP66.Z();
      ignition::math::Vector3d cogP55=links_[4]->WorldCoGPose().Pos()-joints_[4]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T05=ignition::math::Matrix4d(- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), c2*c4 + c3*s2*s4, d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base5= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v05=ignition::math::Vector3d(base5.X(), base5.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v055=T05*v05;
      // cogP55=v055;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP55x=cogP55.X(), cogP55y=cogP55.Y();
      double tau= s6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - c6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) - cogP55y*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - cogP55x*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
#endif
      return tau;
    }

    double gravity_compensation_joint_4(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;
      double g=gravity_constant;
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);

#ifdef OPTION_1
      // wersja nr 1
      double tau=(g*l4*m4*s4*(c1*c3 - c2*s1*s3))/2 - d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - g*m5*s5*(c2*c4 + c3*s2*s4)*(d5 - l5/2);
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y(), cogP66z=cogP66.Z();
      ignition::math::Vector3d cogP55=links_[4]->WorldCoGPose().Pos()-joints_[4]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T05=ignition::math::Matrix4d(- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), c2*c4 + c3*s2*s4, d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base5= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v05=ignition::math::Vector3d(base5.X(), base5.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v055=T05*v05;
      // cogP55=v055;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP55x=cogP55.X(), cogP55y=cogP55.Y(), cogP55z=cogP55.Z();
      ignition::math::Vector3d cogP44=links_[3]->WorldCoGPose().Pos()-joints_[3]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T04=ignition::math::Matrix4d(c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, c3*s1 + c1*c2*s3,   c1*d3*s2, c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c2*s1*s3 - c1*c3,   d3*s1*s2 , c2*s4 - c3*c4*s2, c2*c4 + c3*s2*s4, -s2*s3, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base4= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v04=ignition::math::Vector3d(base4.X(), base4.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v044=T04*v04;
      // std::cout<<"base position="<<base4<<" vector v04="<< v04<<" v044="<<v044<<std::endl;
      // cogP44=v044;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP44x=cogP44.X(), cogP44y=cogP44.Y(), cogP44z=cogP44.Z();
      double tau= s5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) - d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + c5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) - cogP44x*g*m4*(c1*c3 - c2*s1*s3) - cogP44y*g*m4*(c3*s1 + c1*c2*s3);
#endif

      return tau;
    }

    double gravity_compensation_joint_3(){
      //std::cout<<"sin="<<sin(1.57)<<std::endl;
      //s4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + (c4*g*l4*m4*(c1*c3 - c2*s1*s3))/2 + c5*g*m5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) - c4*(g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (c4*g*l4*m4*(c3*s1 + c1*c2*s3))/2 + (g*l4*m4*s2*s3*s4)/2) + c1*g*m3*s2*(d3 - l3/2)

      double g=gravity_constant;
      double m3 = links_[2]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m4 = links_[3]->GetInertial()->Mass(); // because 3-th link in links_ is in fact 4-th link of the lwr4+ manipulator
      double m5 = links_[4]->GetInertial()->Mass(); // because 4-th link in links_ is in fact 5-th link of the lwr4+ manipulator
      double m6 = links_[5]->GetInertial()->Mass(); // because 5-th link in links_ is in fact 6-th link of the lwr4+ manipulator
      double d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);

#ifdef OPTION_1
      // wersja nr 1
      double tau=s4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + (c4*g*l4*m4*(c1*c3 - c2*s1*s3))/2 + c5*g*m5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) - c4*(g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (c4*g*l4*m4*(c3*s1 + c1*c2*s3))/2 + (g*l4*m4*s2*s3*s4)/2) + c1*g*m3*s2*(d3 - l3/2);
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y(), cogP66z=cogP66.Z();
      ignition::math::Vector3d cogP55=links_[4]->WorldCoGPose().Pos()-joints_[4]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T05=ignition::math::Matrix4d(- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), c2*c4 + c3*s2*s4, d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base5= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v05=ignition::math::Vector3d(base5.X(), base5.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v055=T05*v05;
      // cogP55=v055;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP55x=cogP55.X(), cogP55y=cogP55.Y(), cogP55z=cogP55.Z();
      ignition::math::Vector3d cogP44=links_[3]->WorldCoGPose().Pos()-joints_[3]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T04=ignition::math::Matrix4d(c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, c3*s1 + c1*c2*s3,   c1*d3*s2, c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c2*s1*s3 - c1*c3,   d3*s1*s2 , c2*s4 - c3*c4*s2, c2*c4 + c3*s2*s4, -s2*s3, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base4= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v04=ignition::math::Vector3d(base4.X(), base4.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v044=T04*v04;
      // std::cout<<"base position="<<base4<<" vector v04="<< v04<<" v044="<<v044<<std::endl;
      // cogP44=v044;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP44x=cogP44.X(), cogP44y=cogP44.Y(), cogP44z=cogP44.Z();
      ignition::math::Vector3d cogP33=links_[2]->WorldCoGPose().Pos()-joints_[2]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T03=ignition::math::Matrix4d(c1*c2*c3 - s1*s3, - c3*s1 - c1*c2*s3, c1*s2, c1*d3*s2, c1*s3 + c2*c3*s1, c1*c3 - c2*s1*s3, s1*s2, d3*s1*s2 , -c3*s2, s2*s3, c2, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base3= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v03=ignition::math::Vector3d(base3.X(), base3.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v033=T03*v03;
      // cogP33=v033;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP33x=cogP33.X(), cogP33y=cogP33.Y(), cogP33z=cogP33.Z();
      double tau= s4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - c5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) + s5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44z*g*m4*(c1*c3 - c2*s1*s3) - cogP44y*g*m4*s2*s3) - c4*(c6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) - s6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + cogP55y*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP55x*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP44z*g*m4*(c3*s1 + c1*c2*s3) - cogP44x*g*m4*s2*s3) - c1*cogP33y*g*m3*s2 + cogP33x*g*m3*s1*s2;
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
      double d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3, l2=L2;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);

#ifdef OPTION_1
      // wersja nr 1
      double tau=c3*(d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - (g*l4*m4*s4*(c1*c3 - c2*s1*s3))/2 + g*m5*s5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) + d3*(c3*(s4*(g*m4*(c1*c3 - c2*s1*s3) - g*m5*(c2*c4 + c3*s2*s4) - c6*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + g*m6*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c4*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + g*m4*(c3*s1 + c1*c2*s3)) + c1*g*m3*s2) - s3*(s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + g*m3*s1*s2 + g*m4*s2*s3)) + s3*(s4*(g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (c4*g*l4*m4*(c3*s1 + c1*c2*s3))/2 + (g*l4*m4*s2*s3*s4)/2) + c4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + (c4*g*l4*m4*(c1*c3 - c2*s1*s3))/2 + c5*g*m5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) - c2*g*m3*(d3 - l3/2)) + (c1*g*l2*m2*s2)/2;
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y(), cogP66z=cogP66.Z();
      ignition::math::Vector3d cogP55=links_[4]->WorldCoGPose().Pos()-joints_[4]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T05=ignition::math::Matrix4d(- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), c2*c4 + c3*s2*s4, d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base5= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v05=ignition::math::Vector3d(base5.X(), base5.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v055=T05*v05;
      // cogP55=v055;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP55x=cogP55.X(), cogP55y=cogP55.Y(), cogP55z=cogP55.Z();
      ignition::math::Vector3d cogP44=links_[3]->WorldCoGPose().Pos()-joints_[3]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T04=ignition::math::Matrix4d(c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, c3*s1 + c1*c2*s3,   c1*d3*s2, c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c2*s1*s3 - c1*c3,   d3*s1*s2 , c2*s4 - c3*c4*s2, c2*c4 + c3*s2*s4, -s2*s3, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base4= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v04=ignition::math::Vector3d(base4.X(), base4.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v044=T04*v04;
      // std::cout<<"base position="<<base4<<" vector v04="<< v04<<" v044="<<v044<<std::endl;
      // cogP44=v044;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP44x=cogP44.X(), cogP44y=cogP44.Y(), cogP44z=cogP44.Z();
      ignition::math::Vector3d cogP33=links_[2]->WorldCoGPose().Pos()-joints_[2]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T03=ignition::math::Matrix4d(c1*c2*c3 - s1*s3, - c3*s1 - c1*c2*s3, c1*s2, c1*d3*s2, c1*s3 + c2*c3*s1, c1*c3 - c2*s1*s3, s1*s2, d3*s1*s2 , -c3*s2, s2*s3, c2, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base3= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v03=ignition::math::Vector3d(base3.X(), base3.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v033=T03*v03;
      // cogP33=v033;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP33x=cogP33.X(), cogP33y=cogP33.Y(), cogP33z=cogP33.Z();
      ignition::math::Vector3d cogP22=links_[1]->WorldCoGPose().Pos()-joints_[1]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T02=ignition::math::Matrix4d(c1*c2, -c1*s2, -s1, 0, c2*s1, -s1*s2, c1, 0, -s2, -c2, 0, d1, 0, 0,   0,  1);
      // ignition::math::Vector3d base= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v02=ignition::math::Vector3d(base.X(), base.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v022=T02*v02;
      // std::cout<<"base position="<<base<<" vector v02="<< v02<<" v022="<<v022<<std::endl;
      // cogP22=v022;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP22x=cogP22.X(), cogP22y=cogP22.Y(), cogP22z=cogP22.Z();
      double tau= c3*(d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - s5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) - c5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44x*g*m4*(c1*c3 - c2*s1*s3) + cogP44y*g*m4*(c3*s1 + c1*c2*s3) - c2*cogP33x*g*m3 + c1*cogP33z*g*m3*s2) + d3*(c3*(s4*(g*m4*(c1*c3 - c2*s1*s3) - g*m5*(c2*c4 + c3*s2*s4) - c6*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + g*m6*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c4*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + g*m4*(c3*s1 + c1*c2*s3)) + c1*g*m3*s2) - s3*(s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + g*m3*s1*s2 + g*m4*s2*s3)) + s3*(c4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - c5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) + s5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44z*g*m4*(c1*c3 - c2*s1*s3) - cogP44y*g*m4*s2*s3) + s4*(c6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) - s6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + cogP55y*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP55x*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP44z*g*m4*(c3*s1 + c1*c2*s3) - cogP44x*g*m4*s2*s3) + c2*cogP33y*g*m3 - cogP33z*g*m3*s1*s2) + cogP22y*g*m2*s1 + c1*cogP22x*g*m2;
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
      double d5 = D5, d3=D3, d1=D1;
      double l5= L5, l4=L4, l3=L3, l2=L2, l1=L1;


      double theta1=joints_[0]->Position(0);
      double theta2=joints_[1]->Position(0);
      double theta3=joints_[2]->Position(0);
      double theta4=joints_[3]->Position(0);
      double theta5=joints_[4]->Position(0);
      double theta6=joints_[5]->Position(0);

      double c1=cos(theta1), c2=cos(theta2), c3=cos(theta3), c4=cos(theta4), c5=cos(theta5), c6=cos(theta6);
      double s1=sin(theta1), s2=sin(theta2), s3=sin(theta3), s4=sin(theta4), s5=sin(theta5), s6=sin(theta6);


#ifdef OPTION_1
      // wersja nr 1
      double tau=s2*(s3*(d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - (g*l4*m4*s4*(c1*c3 - c2*s1*s3))/2 + g*m5*s5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) + d3*(s3*(s4*(g*m4*(c1*c3 - c2*s1*s3) - g*m5*(c2*c4 + c3*s2*s4) - c6*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + g*m6*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c4*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + g*m4*(c3*s1 + c1*c2*s3)) + c1*g*m3*s2) + c3*(s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + g*m3*s1*s2 + g*m4*s2*s3)) - c3*(s4*(g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (c4*g*l4*m4*(c3*s1 + c1*c2*s3))/2 + (g*l4*m4*s2*s3*s4)/2) + c4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + (c4*g*l4*m4*(c1*c3 - c2*s1*s3))/2 + c5*g*m5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) - c2*g*m3*(d3 - l3/2)) + (c1*c2*g*l2*m2)/2) + c2*(s4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) + (c4*g*l4*m4*(c1*c3 - c2*s1*s3))/2 + c5*g*m5*(c2*c4 + c3*s2*s4)*(d5 - l5/2)) - c4*(g*m5*(d5 - l5/2)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (c4*g*l4*m4*(c3*s1 + c1*c2*s3))/2 + (g*l4*m4*s2*s3*s4)/2) + (c2*g*l2*m2*s1)/2 + c1*g*m3*s2*(d3 - l3/2));
#endif
#ifdef OPTION_2
      // wersja nr 2
      ignition::math::Vector3d cogP66=links_[5]->WorldCoGPose().Pos()-joints_[5]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T06=ignition::math::Matrix4d(- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), - s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base6= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v06=ignition::math::Vector3d(base6.X(), base6.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v066=T06*v06;
      // cogP66=v066;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP66x=cogP66.X(), cogP66y=cogP66.Y(), cogP66z=cogP66.Z();
      ignition::math::Vector3d cogP55=links_[4]->WorldCoGPose().Pos()-joints_[4]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T05=ignition::math::Matrix4d(- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c1*d3*s2, c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), c2*c4 + c3*s2*s4, d1 + c2*d3 + d5*(c2*c4 + c3*s2*s4), 0, 0, 0, 1);
      // ignition::math::Vector3d base5= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v05=ignition::math::Vector3d(base5.X(), base5.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v055=T05*v05;
      // cogP55=v055;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP55x=cogP55.X(), cogP55y=cogP55.Y(), cogP55z=cogP55.Z();
      ignition::math::Vector3d cogP44=links_[3]->WorldCoGPose().Pos()-joints_[3]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T04=ignition::math::Matrix4d(c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, c3*s1 + c1*c2*s3,   c1*d3*s2, c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c2*s1*s3 - c1*c3,   d3*s1*s2 , c2*s4 - c3*c4*s2, c2*c4 + c3*s2*s4, -s2*s3, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base4= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v04=ignition::math::Vector3d(base4.X(), base4.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v044=T04*v04;
      // std::cout<<"base position="<<base4<<" vector v04="<< v04<<" v044="<<v044<<std::endl;
      // cogP44=v044;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP44x=cogP44.X(), cogP44y=cogP44.Y(), cogP44z=cogP44.Z();
      ignition::math::Vector3d cogP33=links_[2]->WorldCoGPose().Pos()-joints_[2]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T03=ignition::math::Matrix4d(c1*c2*c3 - s1*s3, - c3*s1 - c1*c2*s3, c1*s2, c1*d3*s2, c1*s3 + c2*c3*s1, c1*c3 - c2*s1*s3, s1*s2, d3*s1*s2 , -c3*s2, s2*s3, c2, d1 + c2*d3, 0, 0, 0, 1);
      // ignition::math::Vector3d base3= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v03=ignition::math::Vector3d(base3.X(), base3.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v033=T03*v03;
      // cogP33=v033;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP33x=cogP33.X(), cogP33y=cogP33.Y(), cogP33z=cogP33.Z();
      ignition::math::Vector3d cogP22=links_[1]->WorldCoGPose().Pos()-joints_[1]->WorldPose().Pos();
      // matrix $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      // ignition::math::Matrix4d T02=ignition::math::Matrix4d(c1*c2, -c1*s2, -s1, 0, c2*s1, -s1*s2, c1, 0, -s2, -c2, 0, d1, 0, 0,   0,  1);
      // ignition::math::Vector3d base= links_[0]->WorldCoGPose().Pos(); // base position
      // ignition::math::Vector3d v02=ignition::math::Vector3d(base.X(), base.Y(), 0); // lub base.Z()
      // ignition::math::Vector3d v022=T02*v02;
      // std::cout<<"base position="<<base<<" vector v02="<< v02<<" v022="<<v022<<std::endl;
      // cogP22=v022;
      //$$$$$$$$$$$$$$$$$$$$$$
      double cogP22x=cogP22.X(), cogP22y=cogP22.Y(), cogP22z=cogP22.Z();
      ignition::math::Vector3d cogP11=links_[0]->WorldCoGPose().Pos()-joints_[0]->WorldPose().Pos();
      double cogP11x=cogP11.X(), cogP11y=cogP11.Y(), cogP11z=cogP11.Z();
      double tau=c2*(s4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - c5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) + s5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44z*g*m4*(c1*c3 - c2*s1*s3) - cogP44y*g*m4*s2*s3) - c4*(c6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) - s6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + cogP55y*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP55x*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP44z*g*m4*(c3*s1 + c1*c2*s3) - cogP44x*g*m4*s2*s3) + cogP22z*g*m2*s1 - c1*cogP33y*g*m3*s2 + cogP33x*g*m3*s1*s2) + s2*(s3*(d5*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - s5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) - c5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44x*g*m4*(c1*c3 - c2*s1*s3) + cogP44y*g*m4*(c3*s1 + c1*c2*s3) - c2*cogP33x*g*m3 + c1*cogP33z*g*m3*s2) + d3*(s3*(s4*(g*m4*(c1*c3 - c2*s1*s3) - g*m5*(c2*c4 + c3*s2*s4) - c6*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + g*m6*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c4*(s5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + c5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + g*m4*(c3*s1 + c1*c2*s3)) + c1*g*m3*s2) + c3*(s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) + g*m3*s1*s2 + g*m4*s2*s3)) - c3*(c4*(d5*(c5*(g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)) - s5*(g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + g*m6*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)))) - c5*(s6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) + c6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - cogP55z*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP55y*g*m5*(c2*c4 + c3*s2*s4)) + s5*(cogP66y*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - cogP55z*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP66x*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + cogP55x*g*m5*(c2*c4 + c3*s2*s4)) + cogP44z*g*m4*(c1*c3 - c2*s1*s3) - cogP44y*g*m4*s2*s3) + s4*(c6*(cogP66x*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + cogP66z*g*m6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))) - s6*(cogP66y*g*m6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - cogP66z*g*m6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) + cogP55y*g*m5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + cogP55x*g*m5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - cogP44z*g*m4*(c3*s1 + c1*c2*s3) - cogP44x*g*m4*s2*s3) + c2*cogP33y*g*m3 - cogP33z*g*m3*s1*s2) + c1*cogP22z*g*m2);
#endif

      tau=0;
      return tau;
    }

    /* Gravity compensation - algorithm nr 2 */
    void getGravComp2(std::array<double, 7> &t) {
      //
      //
      t[6]=gravity_compensation_joint_7();
      t[5]=gravity_compensation_joint_6();
      t[4]=gravity_compensation_joint_5();
      t[3]=gravity_compensation_joint_4();
      t[2]=gravity_compensation_joint_3();
      t[1]=gravity_compensation_joint_2();
      t[0]=gravity_compensation_joint_1();


      // change the sign of the torque
      std::cout<<std::endl;
      for (int i = 0; i < 7; ++i) {
          t[i] = -t[i];
          std::cout<<"[Optional] t["<<i<<"]="<<t[i]<<std::endl;
      }
      std::cout<<std::endl;
    }

    /*
      gravity compensation - basic algorithm
    */
    void getGravComp(std::array<double, 7 > &t) {
        // gravity vector
        ignition::math::Vector3d gr = gazebo::physics::get_world()->Gravity();
        ignition::math::Vector3d tool_com;
        std::cout<<"tool_com="<<tool_com<<std::endl;

        // tool mass
        double tool_mass = 0;

        // pointer to the last link from the links_ vector - i.e. end effector
        gazebo::physics::LinkPtr link = links_[6];
        // pointer to the last joint from the joints_ vector
        gazebo::physics::JointPtr joint = joints_[6];
        // get the world pose of the link
        ignition::math::Pose3d T_W_L7 = link->WorldPose(); // get the global position of the last link, i.e. in simulation the last link is 6-th link
        std::cout<<"T_W_L7="<<T_W_L7<<std::endl;
        // add to T_W_L7 pose vector tool_com (is it empty?)
        ignition::math::Vector3d cog = T_W_L7.CoordPositionAdd( tool_com ); // calculate the center of gravity of link nr 6 (in fact we number all link from 0 to 6, so in fact here the last link is 6-th but in reality we say it is the 7-th link)
        std::cout<<"tool_com for link 6 => tool_com="<<tool_com<<std::endl;
        std::cout<<"cog for link 6 => cog="<<cog<<std::endl;
        // calculate new vector which is vector cog - position of last joint
        ignition::math::Vector3d r = cog - joint->WorldPose().Pos(); // calculate the distance between the global joint position and the center of gravity (i.e. it's a center of mass)
        std::cout<<"r for link 6 => r="<<r<<std::endl;

        // set a mass to tool_mass - i.e. it equals zero
        double mass = tool_mass; // tool mass we assume equals zero
        std::cout<<"mass="<<mass<<std::endl;
        // calculate torque as a cross product of two vectors r and gravity vector multiplied by mass (which is still zero)
        ignition::math::Vector3d torque = r.Cross(mass * gr); // we calculate the torque exerting on the last joint as a cross product of r and (mass * gr) [arm x mass * gravity constant], pay attention that the mass of the last link is zero
        // calculate axis
        ignition::math::Vector3d axis = joint->GlobalAxis(0); // rotation axis of joint nr 6 in global position
        t[6] = axis.Dot(torque); // dot product of axis and torque is a torque compansating the gravity for the last link

        std::cout<<"#####################################"<<std::endl;
        std::cout<<"Joint position for i=6-th joint: "<<joint->WorldPose().Pos()<<std::endl;
        std::cout<<"Axis for i=6-th joint: "<<axis<<std::endl;
        std::cout<<"Torque for i=6-th joint: "<<t[6]<<std::endl;

        // for each link within links_ - except the 6-th link
        for (int i = 6; i > 0; i--) {
            link = links_[i-1]; // get the (i-1)th link
            joint = joints_[i-1]; // get the (i-1)th joint
            // WorldCoGPose - get the pose of the body's center of gravity in the world coordinate frame
            // now we calculate the center of gravity for all links already visited, i.e. (i-1) to 6, we are using weighted mean
            cog = (cog * mass + link->WorldCoGPose().Pos() * link->GetInertial()->Mass()) / (mass+link->GetInertial()->Mass()); // we calculate here the weighted mean, based on this we calculate the center of gravity of the links (from i-1-th link do 6-th link)
            std::cout<<"cog="<<cog<<std::endl;

            // update the total mass of already visited links, i.e. (i-1) to 6
            mass += link->GetInertial()->Mass(); // here we calculate the total sum of the manipulator (iteratively adding masses of links starting from end-effector, which has zero mass)
            std::cout<<"mass["<<i-1<<"]="<<mass<<std::endl;

            // caluclate the distance between the joint position and the center of gravity of all already visited joints
            r = cog - joint->WorldPose().Pos();
            // calculate the torque excerting on joint, as a cross product of arm and the gravitation force acting on the arm
            torque = r.Cross(mass * gr);

            // global axis of joint (i-1) - i.e. rotation axis of joint, in other words the z-th vector from transformation matrix ^0_(i-1)T
            axis = joint->GlobalAxis(0);

            // torque exerting on joint (i-1) along, why dot product? because we calculate torque along z-th axis from transformation matrix (rotation axis of joint i-1 from poiint of view of base frame)
            t[i-1] = axis.Dot(torque);

            std::cout<<"#####################################"<<std::endl;
            std::cout<<"Joint position for i="<<i-1<<"-th joint "<<joint->WorldPose().Pos()<<std::endl;
            std::cout<<"Axis for i="<<i-1<<"-th joint "<<axis<<std::endl;
            std::cout<<"Torque for i="<<i-1<<"-th joint "<<t[i-1]<<std::endl;

        }

        std::cout<<"#####################################"<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<"#####################################"<<std::endl;

        // change the sign of the torque
        for (int i = 0; i < 7; ++i) {
            t[i] = -t[i];
            std::cout<<"[Original] t["<<i<<"]="<<t[i]<<std::endl;
        }
    } // end of function getGravComp

    // Called by the world update start event
    public: void OnUpdate()
    {
        // in every iteration:
        // reset torque values
        std::array<double, 7 > t;
        t.fill(0);      // initialize all torques to 0

        // get gravity compensation torques - our function
        getGravComp(t);
        t.fill(0);      // initialize all torques to 0
        getGravComp2(t);

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
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        double k = 0.1;    // stiffness constant
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
