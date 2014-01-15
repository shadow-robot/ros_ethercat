 /*
 * ros_ethercat.cpp
 *
 *  Created on: 7 Jan 2014
 *      Author: Manos Nikolaidis
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Shadow Robot Company Ltd.
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

#include "ros_ethercat/ros_ethercat.hpp"
#include "ros/console.h"

using std::map;
using std::string;
using pr2_mechanism_model::JointState;
using pr2_mechanism_model::RobotState;
using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;

bool ros_ethercat::initXml(TiXmlElement* config)
{
  if (!model_.initXml(config))
  {
    ROS_ERROR("Failed to initialize pr2 mechanism model");
    return false;
  }
  state_ = new RobotState(&model_);

  for (map<string, JointState*>::const_iterator it = state_->joint_states_map_.begin(); it != state_->joint_states_map_.end(); ++it)
  {
    JointStateHandle jsh(it->first, &it->second->position_, &it->second->velocity_, &it->second->measured_effort_);
    joint_state_interface_.registerHandle(jsh);

    JointHandle jh(joint_state_interface_.getHandle(it->first), &it->second->commanded_effort_);
    joint_command_interface_.registerHandle(jh);
    effort_joint_interface_.registerHandle(jh);
  }

  registerInterface(state_);
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_command_interface_);
  registerInterface(&effort_joint_interface_);

  return true;
}

void ros_ethercat::read()
{
  state_->propagateActuatorPositionToJointPosition();
  state_->zeroCommands();
}

void ros_ethercat::write()
{
  state_->enforceSafety();
  state_->propagateJointEffortToActuatorEffort();
}
