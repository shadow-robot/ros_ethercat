/*
* power_delivery_state.hpp
*
*  Created on: 26 Sept 2025
*      Author: Chris White
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025 Shadow Robot Company Ltd.
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

/*
* @file   power_delivery_state.h
* @author Chris White <chris@shadowrobot.com>
* @brief  Hardware interface for power delivery monitoring
*/

#ifndef ROS_ETHERCAT_MODEL_POWERDELIVERYSTATE_HPP
#define ROS_ETHERCAT_MODEL_POWERDELIVERYSTATE_HPP

#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_model/hardware_interface.hpp"
#include "ros_ethercat_model/robot_state.hpp"
#include <map>
#include <string>
#include <vector>
#include <hardware_interface/internal/hardware_resource_manager.h>

using std::vector;
using std::string;


namespace ros_ethercat_model
{

class PowerDeliveryState : public hardware_interface::HardwareInterface
{
public:
  PowerDeliveryState() : name_(), dimensions_(0)
  {
  }

  explicit PowerDeliveryState(string name) : name_(name), dimensions_(0)
  {
  }

  void setName(string name)
  {
    name_ = name;
  }


  string getName() const { return name_; }

  uint16_t v_4v;
  uint16_t i_4v;
  uint16_t v_5v5;
  uint16_t i_5v5;
  uint16_t v_24v;
  uint16_t i_24v;

private:
  string name_;
  size_t dimensions_;
};

}  // namespace ros_ethercat_model

namespace hardware_interface
{

class PowerDeliveryStateHandle
{
public:
  PowerDeliveryStateHandle() : name_(), state_(0) {}
  PowerDeliveryStateHandle(string name, ros_ethercat_model::PowerDeliveryState* state) : name_(name), state_(state)
  {
    if (!state)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + 
        "'. PowerDeliveryState state data pointer is null.");
    }
  }
  string getName() const {return name_;}
  ros_ethercat_model::PowerDeliveryState* getState() const
  {
    assert(state_);
    return state_;
  }
private:
  string name_;
  ros_ethercat_model::PowerDeliveryState* state_;
};

class PowerDeliveryStateInterface : public HardwareResourceManager<hardware_interface::PowerDeliveryStateHandle>
{
};



}  // namespace hardware_interface


#endif  // ROS_ETHERCAT_MODEL_POWERDELIVERYSTATE_HPP
