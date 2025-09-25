/*
* Copyright (C) 2025 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

/*
* @file   temperature_sensor_state.h
* @author Chris White <chris@shadowrobot.com>
* @brief  Hardware interface for temperature sensor
*/

#ifndef ROS_ETHERCAT_MODEL_TEMPERATURESENSORSTATE_HPP
#define ROS_ETHERCAT_MODEL_TEMPERATURESENSORSTATE_HPP

#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_model/hardware_interface.hpp"
#include "ros_ethercat_model/robot_state.hpp"
#include <map>
#include <string>
#include <vector>
#include <hardware_interface/internal/hardware_resource_manager.h>

/* \brief This class provides the controllers with an interface to the reference temperature sensor states
*
*/

using std::vector;
using std::string;


namespace ros_ethercat_model
{

class TempSensorState : public hardware_interface::HardwareInterface
{
public:
  TempSensorState() : name_(), dimensions_(0)
  {
  }

  explicit TempSensorState(string name) : name_(name), dimensions_(0)
  {
  }

  void setName(string name)
  {
    name_ = name;
  }


  string getName() const { return name_; }

  uint16_t temperature_;
  
private:
  string name_;
  size_t dimensions_;
};

}  // namespace ros_ethercat_model

namespace hardware_interface
{

class TempSensorStateHandle
{
public:
  TempSensorStateHandle() : name_(), state_(0) {}
  TempSensorStateHandle(string name, ros_ethercat_model::TempSensorState* state) : name_(name), state_(state)
  {
    if (!state)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. TempSensor state data pointer is null.");
    }
  }
  string getName() const {return name_;}
  ros_ethercat_model::TempSensorState* getState() const
  {
    assert(state_);
    return state_;
  }
private:
  string name_;
  ros_ethercat_model::TempSensorState* state_;
};

class TempSensorStateInterface : public HardwareResourceManager<hardware_interface::TempSensorStateHandle>
{
};



}  // namespace hardware_interface


#endif  // ROS_ETHERCAT_MODEL_TEMPERATURESENSORSTATE_HPP
