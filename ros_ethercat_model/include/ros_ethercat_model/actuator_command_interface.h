/*
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Shadow Robot Company Ltd.
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

#ifndef ROS_ETHERCAT_MODEL_ACTUATOR_COMMAND_INTERFACE_H
#define ROS_ETHERCAT_MODEL_ACTUATOR_COMMAND_INTERFACE_H

#include <hardware_interface/actuator_command_interface.h>
#include <ros_ethercat_model/hardware_interface.hpp>

#include <string>
#include <vector>


namespace ros_ethercat_model
{

/** \brief A handle used to read and command a single actuator. */
class ActuatorHandle : public hardware_interface::ActuatorHandle
{
public:
  ActuatorHandle() : hardware_interface::ActuatorHandle(), cmd_(0), cmd_type_(0) {}
  ActuatorHandle(const ActuatorStateHandle& as, double* cmd, ActuatorCommandMode* cmd_type = 0)
    : hardware_interface::ActuatorHandle(as, cmd), cmd_(cmd), cmd_type_(cmd_type) {}


  void setCommand(double command, ActuatorCommandMode command_type)
  {
    assert(cmd_); *cmd_ = command;
    *cmd_type_ = command_type;
  }

  void setState(ActuatorState* as)
  {
    a_ = as;
  }

  ActuatorState * getState(void)
  {
    assert(a_);
    return a_;
  }

private:
  ActuatorState *a_;
  double* cmd_;
  ActuatorCommandMode* cmd_type_;
};

class ActuatorCommandInterface : public hardware_interface::HardwareResourceManager<ActuatorHandle> {};

/// \ref ActuatorCommandInterface for commanding effort-based actuators
class EffortActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding velocity-based actuators
class VelocityActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding position-based actuators
class PositionActuatorInterface : public ActuatorCommandInterface {};


}  // namespace ros_ethercat_model

#endif  // ROS_ETHERCAT_MODEL_ACTUATOR_COMMAND_INTERFACE_H
