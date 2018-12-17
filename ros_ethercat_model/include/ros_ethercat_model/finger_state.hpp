/*
 * finger_state.hpp
 *
 *  Created on: 11 Dec 2018
 *      Author: Daniel Greenwald
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef ROS_ETHERCAT_MODEL_FINGERSTATE_HPP
#define ROS_ETHERCAT_MODEL_FINGERSTATE_HPP

#include <pluginlib/class_loader.h>
#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_model/joint.hpp"

#include "ros_ethercat_model/hardware_interface.hpp"
#include <map>
#include <string>


/** \brief This class provides the controllers with an interface to the finger state
 *
 * Most controllers that need the robot state should use the joint states, to get
 * access to the joint position/velocity/effort, and to command the effort a joint
 * should apply. Controllers can get access to the hard realtime clock through current_time_
 */

using std::vector;
using std::string;

namespace ros_ethercat_model
{
  
class FingerState : public hardware_interface::HardwareInterface
{
public:
  FingerState() : name_() {}
  FingerState(string name, vector <JointState*> joint_states, vector<ActuatorState*> actuator_states)
    : name_(name), joint_states_(joint_states), actuator_states_(actuator_states)
  {

  }

  FingerState(string name) : name_(name)
  {

  }

  void addActuatorState(ActuatorState* actuator_state)
  {
    actuator_states_.push_back(actuator_state);
  }

  void addJointState(JointState* joint_state)
  {
    joint_states_.push_back(joint_state);
  }

  string getName() const { return name_;  }

  vector<JointState*> joint_states_;
  vector<ActuatorState*> actuator_states_;

private:
  string name_;

};

//class FingerStateInterface : public hardware_interface::HardwareResourceManager <FingerState> {};

class FingerStateHandle
{
public:
  FingerStateHandle() : name_(), state_(0) {}
  FingerStateHandle(string name, FingerState* state) : name_(name), state_(state)
  {
    if (!state)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Finger state data pointer is null.");
    }
    
  }
  string getName() const {return name_;}
  FingerState* getState() const {assert(state_); return state_;}
private:
  string name_;
  FingerState* state_;
 
};

class FingerStateInterface : public hardware_interface::HardwareResourceManager<FingerStateHandle> {};
  
} 
#endif
