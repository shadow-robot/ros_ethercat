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

#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

namespace ros_ethercat_hardware_interface{


class ActuatorState{
public:
  ActuatorState() :
      timestamp_(0),
      device_id_(0),
      encoder_count_(0),
      position_(0),
      encoder_velocity_(0),
      velocity_(0),
      calibration_reading_(0),
      calibration_rising_edge_valid_(0),
      calibration_falling_edge_valid_(0),
      last_calibration_rising_edge_(0),
      last_calibration_falling_edge_(0),
      is_enabled_(0),
      halted_(0),
      last_commanded_current_(0),
      last_executed_current_(0),
      last_measured_current_(0),
      last_commanded_effort_(0),
      last_executed_effort_(0),
      last_measured_effort_(0),
      motor_voltage_(0),
      num_encoder_errors_(0),
      zero_offset_(0)
  {}

  /**
   * The time at which actuator state was measured, relative to the time the ethercat process was started.
   * Timestamp value is not synchronised with wall time and may be different for different actuators.
   * For Willow Garage motor controllers, timestamp is made when actuator data is sampled.
   * sample_timestamp_ will provide better accuracy than ros::Time::now() or robot->getTime()
   * when using a time difference in calculations based on actuator variables.
   */
  ros::Duration sample_timestamp_;

  /** The time at which this actuator state was measured (in seconds).
   * This value should be same as sample_timestamp_.toSec() for Willow Garage devices.
   * The timestamp_ variable is being kept around for backwards compatibility, new controllers
   * should use sample_timestamp_ instead.
   */
  double timestamp_;

  int device_id_; //!< Position in EtherCAT chain

  int encoder_count_; //!< The number of ticks as reported by the encoder
  double position_; //!< The position of the motor (in radians)
  double encoder_velocity_; //!< The velocity measured in encoder ticks per second
  double velocity_; //!< The velocity in radians per second

  bool calibration_reading_; //!< the value of the last calibration reading: low (false) or high (true)
  bool calibration_rising_edge_valid_; //!< Is the last_callibration_rising_edge_ field valid?
  bool calibration_falling_edge_valid_; //!< Is the last_callibration_falling_edge_ field valid?
  double last_calibration_rising_edge_; //!< The position of the motor the last time the calibration switch went from low to high.
  double last_calibration_falling_edge_; //!< The position of the motor the last time the calibration switch went from high to low.

  bool is_enabled_; //!< Enable status
  bool halted_; //!< indicates if the motor is halted. A motor can be halted because of voltage or communication problems

  double last_commanded_current_; //!< The current computed based on the effort specified in the ActuatorCommand (in amps)
  double last_executed_current_; //!< The actual current requested after safety limits were enforced (in amps)
  double last_measured_current_; //!< The measured current (in amps)

  double last_commanded_effort_; //!< The torque requested in the previous ActuatorCommand (in Nm)
  double last_executed_effort_; //!< The torque applied after safety limits were enforced (in Nm)
  double last_measured_effort_; //!< The measured torque (in Nm)

  double max_effort_; //!< Absolute torque limit for actuator (derived from motor current limit). (in Nm)

  double motor_voltage_; //!< Motor voltage (in volts)

  int num_encoder_errors_; //!< The number of invalid encoder signal transitions

  double zero_offset_; //!< A bias applied to the position value when reported.  This value is written once after calibration. The reported position is the hardware's actual position minus the zero offset
};

class ActuatorCommand
{
public:
  ActuatorCommand() :
    enable_(0), effort_(0)
  {}
  bool enable_; //!< Enable this actuator
  double effort_; //!< Force to apply (in Nm)
};

/*!
 * \class Actuator
 * The Actuator class provides an interface for the PR2's motor controller
 * board (MCB).  The MCB is connected to a motor, encoder, and optional
 * calibration sensor.
 *
 * The ActuatorCommand class is used to enable the motor and set the commanded
 * efforts of the motor (in Nm).
 *
 * The ActuatorState class reports back on the state of the motor, encoder,
 * and calibration sensor.
 */
class Actuator
{
public:
  Actuator() {};
  Actuator(std::string name) : name_(name) {}
  std::string name_;
  ActuatorState state_;
  ActuatorCommand command_;
};

/*!
 * \class CustomHW
 * The CustomHW class provides an easy way to add more hardware to the HardwareInterface.
 * Simply inherit from that class to add a new type of hardware, containing the data you
 * want in its command and state.
 */
class CustomHW
{
public:
  std::string name_;
};

typedef std::map<std::string, Actuator*> ActuatorMap;
typedef std::map<std::string, CustomHW*> CustomHWMap;

/*!
 * \class HardwareInterface
 * The HardwareInterface class provides access to the PR2 hardware
 * components that are controlled via EtherCAT.  These components include:
 *  - Actuators
 *  - CustomHW
 *
 * For each component type, there exists a class definition that consists of
 * the following three fields:
 *  # name - A unique name for this instance of a component type
 *  # command - A class which is used to send commands to this component
 *  # status - A class which is used to return the status of this component
 *
 * Drivers that provide one or more of these components register the
 * corresponding class for that component by name with the HardwareInterface.
 * For a given component type, names must be unique.
 *
 * Controllers can retrieve a pointer to a component's class by name.  The
 * component is controlled using the command_ field of the component class,
 * and its status is given in the status_ field.
 */
class HardwareInterface
{
public:
  ActuatorMap actuators_;
  CustomHWMap custom_hws_;
  /*! \brief Get a pointer to the actuator by name
   *
   *  \param name The name of the actuator
   *  \return A pointer to an Actuator.  Returns NULL if name is not valid.
   */
  Actuator* getActuator(const std::string &name) const {
    ActuatorMap::const_iterator it = actuators_.find(name);
    return it != actuators_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the Custom Hardware device by name
   *
   *  \param name The name of the Custom Hardware device
   *  \return A pointer to a CustomHW.  Returns NULL if name is not valid.
   */
  CustomHW* getCustomHW(const std::string &name) const {
    CustomHWMap::const_iterator it = custom_hws_.find(name);
    return it != custom_hws_.end() ? it->second : NULL;
  }

  /*! \brief Add an actuator to the hardware interface
   *
   *  \param actuator A pointer to the Actuator
   *  \return true if successful, false if name is a duplicate
   */
  bool addActuator(Actuator *actuator) {
    std::pair<ActuatorMap::iterator, bool> p;
    p = actuators_.insert(ActuatorMap::value_type(actuator->name_, actuator));
    return p.second;
  }

  /*! \brief Add a Custom Hardware device to the hardware interface
   *
   *  \param custom_hw A pointer to the CustomHW
   *  \return true if successful, false if name is a duplicate
   */
  bool addCustomHW(CustomHW *custom_hw) {
    std::pair<CustomHWMap::iterator, bool> p;
    p = custom_hws_.insert(CustomHWMap::value_type(custom_hw->name_, custom_hw));
    return p.second;
  }

  ros::Time current_time_; //!< The time at which the commands were sent to the hardware
};
}

#endif
