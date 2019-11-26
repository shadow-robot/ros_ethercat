/*
* Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef ROS_ETHERCAT_MODEL_ACTUATOR_COMMAND_INTERFACE_H
#define ROS_ETHERCAT_MODEL_ACTUATOR_COMMAND_INTERFACE_H

#include <hardware_interface/actuator_command_interface.h>
#include <string>
#include <vector>

namespace ros_ethercat_model
{

typedef enum ActuatorCommandMode
{
  COMMAND_TYPE_PWM = 0,
  COMMAND_TYPE_EFFORT = 1
}
ActuatorCommandMode;

inline std::vector<std::string> command_types_to_string()
{
  std::vector<std::string> command_type_strings;

  command_type_strings.push_back("pwm");
  command_type_strings.push_back("effort");

  return command_type_strings;
}


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

private:
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
