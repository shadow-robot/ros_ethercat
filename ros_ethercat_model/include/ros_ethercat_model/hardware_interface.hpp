/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ROS_ETHERCAT_MODEL_HARDWARE_INTERFACE_H
#define ROS_ETHERCAT_MODEL_HARDWARE_INTERFACE_H

#include <string>
#include <vector>

#include <ros/ros.h>

namespace ros_ethercat_model
{

class ActuatorState
{
public:
  ActuatorState() :
    device_id_(0),
    position_(0),
    velocity_(0),
    last_commanded_current_(0.0),
    last_measured_current_(0.0),
    last_commanded_effort_(0.0),
    last_measured_effort_(0.0),
    max_effort_(0.0),
    motor_voltage_(0.0)
  {
  }

  int device_id_; //!< Position in EtherCAT chain

  double position_; //!< The position of the motor (in radians)
  double velocity_; //!< The velocity in radians per second

  double last_commanded_current_; //!< Current computed based on effort specified in ActuatorCommand (in amps)
  double last_measured_current_; //!< The measured current (in amps)

  double last_commanded_effort_; //!< The torque requested in the previous ActuatorCommand (in Nm)
  double last_measured_effort_; //!< The measured torque (in Nm)

  double max_effort_; //!< Absolute torque limit for actuator (derived from motor current limit). (in Nm)

  double motor_voltage_; //!< Motor voltage (in volts)
};

class ActuatorCommand
{
public:
  ActuatorCommand() :
    enable_(0),
    effort_(0)
  {
  }

  bool enable_; //!< Enable this actuator
  double effort_; //!< Force to apply (in Nm)
};

/*!
 * \class Actuator
 * The Actuator class provides an interface for the motor controller
 *
 * The ActuatorCommand class is used to enable the motor and set the commanded
 * efforts of the motor (in Nm).
 *
 * The ActuatorState class reports back on the state of the motor
 */
class Actuator
{
public:
  std::string name_;
  ActuatorState state_;
  ActuatorCommand command_;
};

/*!
 * \class CustomHW
 * The CustomHW class provides an easy way to add more hardware to the HardwareInterface.
 * Inherit from that class to add a new type of hardware, containing the data you want.
 * If the hardware doesn't use EtherCAT, constructor and destructor
 * should initialize drivers to communicate with hardware and
 * read and write functions must be implemented accordingly.
 * The read and write functions will be called by RosEthercat functions with same names
 */
class CustomHW
{
public:
  virtual ~CustomHW() {}
  virtual void read(const ros::Time &time) {}
  virtual void write(const ros::Time &time) {}
};

}

#endif
