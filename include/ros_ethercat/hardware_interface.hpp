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

class PressureSensorCommand
{
};

class PressureSensorState
{
public:
  std::vector<uint16_t> data_; //!< A vector of 22 pressure readings from each sensor.
};

/*!
 * \class PressureSensor
 * Each gripper on the PR2 has two pressure sensitive finger tips.
 *
 * The pressure sensors are not configurable under software control, thus
 * the PressureSensorCommand class is empty.
 *
 * The state of thre pressure sensorts is reported back in the
 * PressureSensorState class.  Each finger tip reports back 22 16-bit values.
 */
class PressureSensor
{
public:
  std::string name_;
  PressureSensorState state_;
  PressureSensorCommand command_;
};

class AccelerometerCommand
{
public:
  enum {RANGE_2G=0, RANGE_4G=1, RANGE_8G=2}; //! Enums for possible accelerometer range settings.
  enum {BANDWIDTH_1500HZ=6, BANDWIDTH_750HZ=5, BANDWIDTH_375HZ=4, BANDWIDTH_190HZ=3, BANDWIDTH_100HZ=2, BANDWIDTH_50HZ=1, BANDWIDTH_25HZ=0}; //! Enums for possible accelerometer bandwidth settings.
  AccelerometerCommand() : range_(RANGE_2G), bandwidth_(BANDWIDTH_1500HZ) {}
  int range_; //!< The range of the values to be returned (range of 0 means within +/- 2g, 1 means within +/-4g, 2 means /- 8g).  Value is reported in m/s/s.
  int bandwidth_; //!< Accelerometer bandwidth setting. Value is passed directly to Bosch accelerometer (BMA 150). The maximum bandwidth of 1500Hz is appropriate in almost all cases.  Instead of changing bandwidth value, use software filter to remove high frequencies from accelerometer data.  This way, other accerometer users are not affected.  Read accelerometer datasheet for more information about possible bandwidth setting.
};

class AccelerometerState
{
public:
  std::string frame_id_;  //!< Frame id of accelerometer
  std::vector<geometry_msgs::Vector3> samples_; //!< A vector of samples taken from the accelerometer (in m/s/s) since the last iteration of the control loop (oldest samples first).
};

/*!
 * \class Accelerometer
 * Some of the PR2's motor controller boards (MCBs) contain 3 axis
 * accelerometers.
 *
 * The AccelerometerCommand class allows the user to configure the
 * accelerometer's range (from 2-8 Gs) and the bandwidth.
 *
 * The accelerometer's state is returned in the AccelerometerState class.
 */
class Accelerometer
{
public:
  std::string name_;
  AccelerometerState state_;
  AccelerometerCommand command_;
};

class ForceTorqueState
{
public:
  bool good_; //!< True if sensor is working properly. False if some type of error is detected.
  //!< A vector of samples taken from force/torque sensor since the last iteration of the control loop (oldest samples first).
  std::vector<geometry_msgs::Wrench> samples_;
};

class ForceTorqueCommand
{
public:
  //! If halt_on_error_ is true, the driver with halt motors when an there is an error is detected.  The default setting is false.
  bool halt_on_error_;
};

class ForceTorque
{
public:
  std::string name_;
  ForceTorqueState state_;
  ForceTorqueCommand command_;
};


class DigitalOutCommand
{
public:
  DigitalOutCommand() : data_(false) {}
  DigitalOutCommand(bool data) : data_(data) {}
  uint8_t data_;
};

class DigitalOutState
{
public:
  uint8_t data_;
};

/*!
 * \class DigitalOut
 * The PR2's motor controller boards have one or more digital I/O pins.
 * The digital I/Os can be used to control a variety of hardware such as
 * a calibration LED or a camera trigger.
 *
 * The DigitalOutCommand class sets the state of the digital I/O.
 *
 * The DigitalOutState class returns the state of the digital I/O.
 */
class DigitalOut
{
public:
  std::string name_;
  DigitalOutState state_;
  DigitalOutCommand command_;
};

class ProjectorCommand
{
public:
  ProjectorCommand(uint8_t &A, uint8_t &B, uint8_t &I, uint8_t &M, uint8_t &L0, uint8_t &L1) : enable_(0), current_(0), A_(A), B_(B), I_(I), M_(M), L0_(L0), L1_(L1) {}
  bool enable_;
  double current_;
  uint8_t &A_;
  uint8_t &B_;
  uint8_t &I_;
  uint8_t &M_;
  uint8_t &L0_;
  uint8_t &L1_;
  bool pulse_replicator_;
};

class ProjectorState
{
public:
  ProjectorState(uint8_t &A, uint8_t &B, uint8_t &I, uint8_t &M, uint8_t &L0, uint8_t &L1) : A_(A), B_(B), I_(I), M_(M), L0_(L0), L1_(L1) {}
  bool enable_;
  double last_commanded_current_;
  double last_executed_current_;
  double last_measured_current_;
  double max_current_;  //!< Current limit (Amps).  Minimum of board and LED limits.
  uint8_t &A_;
  uint8_t &B_;
  uint8_t &I_;
  uint8_t &M_;
  uint8_t &L0_;
  uint8_t &L1_;
  bool pulse_replicator_;

  bool output_;
  bool rising_timestamp_valid_;
  bool falling_timestamp_valid_;

  uint32_t timestamp_us_;
  uint32_t rising_timestamp_us_;
  uint32_t falling_timestamp_us_;
};

/*!
 * \class Projector
 * The PR2 has a textured-light projector for better stereo imaging. While
 * this textured light improves the quality of 3D reconstruction from stereo
 * images, it is not desirable to have the projector running for other imagers
 * on PR2.  The PR2's projector has a complicated system of controlling
 * its state based on several external triggers.
 *
 * The projector exposes its interface in two separate ways:
 *  - As a ProjectorCommand/ProjectorState pair
 *  - As a collection of 6 DigitalOut instances
 *
 * The ProjectorCommand class allows the projector board to be configured
 * for a variety of operational modes.
 *
 * The ProjectorState class reports on the current status of the projector
 * board.
 */
class Projector
{
public:
  Projector(DigitalOut &A, DigitalOut &B, DigitalOut &I, DigitalOut &M, DigitalOut &L0, DigitalOut &L1) : state_(A.state_.data_, B.state_.data_, I.state_.data_, M.state_.data_, L0.state_.data_, L1.state_.data_), command_(A.command_.data_, B.command_.data_, I.command_.data_, M.command_.data_, L0.command_.data_, L1.command_.data_) {}
  std::string name_;
  ProjectorState state_;
  ProjectorCommand command_;
};

class AnalogInCommand
{
};

class AnalogInState
{
public:
  std::vector<double> state_;
};

class AnalogIn
{
public:
  std::string name_;
  AnalogInState state_;
  AnalogInCommand command_;
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
typedef std::map<std::string, PressureSensor*> PressureSensorMap;
typedef std::map<std::string, Accelerometer*> AccelerometerMap;
typedef std::map<std::string, ForceTorque*> ForceTorqueMap;
typedef std::map<std::string, DigitalOut*> DigitalOutMap;
typedef std::map<std::string, Projector*> ProjectorMap;
typedef std::map<std::string, AnalogIn*> AnalogInMap;
typedef std::map<std::string, CustomHW*> CustomHWMap;

/*!
 * \class HardwareInterface
 * The HardwareInterface class provides access to the PR2 hardware
 * components that are controlled via EtherCAT.  These components include:
 *  - Actuators
 *  - Finger-tip Pressure Sensors
 *  - Accelerometers
 *  - Force/Torque Sensors
 *  - Digital I/Os
 *  - Projectors
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
  PressureSensorMap pressure_sensors_;
  AccelerometerMap accelerometers_;
  ForceTorqueMap ft_sensors_;
  DigitalOutMap digital_outs_;
  ProjectorMap projectors_;
  AnalogInMap analog_ins_;
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

  /*! \brief Get a pointer to the pressure sensor by name
   *
   *  \param name The name of the pressure sensor
   *  \return A pointer to a PressureSensor.  Returns NULL if name is not valid.
   */
  PressureSensor* getPressureSensor(const std::string &name) const {
    PressureSensorMap::const_iterator it = pressure_sensors_.find(name);
    return it != pressure_sensors_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the accelerometer by name
   *
   *  \param name The name of the accelerometer
   *  \return A pointer to an Accelerometer.  Returns NULL if name is not valid.
   */
  Accelerometer* getAccelerometer(const std::string &name) const {
    AccelerometerMap::const_iterator it = accelerometers_.find(name);
    return it != accelerometers_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the FT sensor by name
   *
   *  \param name The name of the FT sensor
   *  \return A pointer to a FT sensor.  Returns NULL if name is not valid.
   */
  ForceTorque* getForceTorque(const std::string &name) const {
    ForceTorqueMap::const_iterator it = ft_sensors_.find(name);
    return it != ft_sensors_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the digital I/O by name
   *
   *  \param name The name of the digital I/O
   *  \return A pointer to an DigitalOut.  Returns NULL if name is not valid.
   */
  DigitalOut* getDigitalOut(const std::string &name) const {
    DigitalOutMap::const_iterator it = digital_outs_.find(name);
    return it != digital_outs_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the projector by name
   *
   *  \param name The name of the projector
   *  \return A pointer to an Projector.  Returns NULL if name is not valid.
   */
  Projector* getProjector(const std::string &name) const {
    ProjectorMap::const_iterator it = projectors_.find(name);
    return it != projectors_.end() ? it->second : NULL;
  }

  /*! \brief Get a pointer to the analog-in device by name
   *
   *  \param name The name of the analog-in device
   *  \return A pointer to an AnalogIn.  Returns NULL if name is not valid.
   */
  AnalogIn* getAnalogIn(const std::string &name) const {
    AnalogInMap::const_iterator it = analog_ins_.find(name);
    return it != analog_ins_.end() ? it->second : NULL;
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

  /*! \brief Add an pressure sensor to the hardware interface
   *
   *  \param sensor A pointer to the PressureSensor
   *  \return true if successful, false if name is a duplicate
   */
  bool addPressureSensor(PressureSensor *sensor) {
    std::pair<PressureSensorMap::iterator, bool> p;
    p = pressure_sensors_.insert(PressureSensorMap::value_type(sensor->name_, sensor));
    return p.second;
  }

  /*! \brief Add an accelerometer to the hardware interface
   *
   *  \param accelerometer A pointer to the Accelerometer
   *  \return true if successful, false if name is a duplicate
   */
  bool addAccelerometer(Accelerometer *accelerometer) {
    std::pair<AccelerometerMap::iterator, bool> p;
    p = accelerometers_.insert(AccelerometerMap::value_type(accelerometer->name_, accelerometer));
    return p.second;
  }

  /*! \brief Add a FT sensor to the hardware interface
   *
   *  \param forcetorque A pointer to the ForceTorque
   *  \return true if successful, false if name is a duplicate
   */
  bool addForceTorque(ForceTorque *forcetorque) {
    std::pair<ForceTorqueMap::iterator, bool> p;
    p = ft_sensors_.insert(ForceTorqueMap::value_type(forcetorque->name_, forcetorque));
    return p.second;
  }

  /*! \brief Add an digital I/O to the hardware interface
   *
   *  \param digital_out A pointer to the DigitalOut
   *  \return true if successful, false if name is a duplicate
   */
  bool addDigitalOut(DigitalOut *digital_out) {
    std::pair<DigitalOutMap::iterator, bool> p;
    p = digital_outs_.insert(DigitalOutMap::value_type(digital_out->name_, digital_out));
    return p.second;
  }

  /*! \brief Add an projector to the hardware interface
   *
   *  \param projector A pointer to the Projector
   *  \return true if successful, false if name is a duplicate
   */
  bool addProjector(Projector *projector) {
    std::pair<ProjectorMap::iterator, bool> p;
    p = projectors_.insert(ProjectorMap::value_type(projector->name_, projector));
    return p.second;
  }

  /*! \brief Add an analog-in device to the hardware interface
   *
   *  \param analog_in A pointer to the AnalogIn
   *  \return true if successful, false if name is a duplicate
   */
  bool addAnalogIn(AnalogIn *analog_in) {
    std::pair<AnalogInMap::iterator, bool> p;
    p = analog_ins_.insert(AnalogInMap::value_type(analog_in->name_, analog_in));
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
