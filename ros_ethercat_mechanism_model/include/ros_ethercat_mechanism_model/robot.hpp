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

/*
 * The robot model tracks the state of the robot.
 *
 * State path:
 *               +---------------+
 * Actuators --> | Transmissions | --> Joints
 *               +---------------+
 *
 * Author: Stuart Glaser
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <urdf/model.h>
#include <pluginlib/class_loader.h>
#include <boost/unordered_map.hpp>
#include <ros_ethercat_hardware_interface/hardware_interface.hpp>
#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_mechanism_model/joint.hpp"
#include "ros_ethercat_mechanism_model/transmission.hpp"

class TiXmlElement;

// Forward declared to avoid extra includes
namespace pluginlib {
template <class T> class ClassLoader;
}

namespace ros_ethercat_mechanism_model
{

/** \brief This class provides the controllers with an interface to the robot state
 *
 * Most controllers that need the robot state should use the joint states, to get
 * access to the joint position/velocity/effort, and to command the effort a joint
 * should apply. Controllers can get access to the hard realtime clock through getTime()
 *
 * Some specialized controllers (such as the calibration controllers) can get access
 * to actuator states, and transmission states.
 */
class Robot : public hardware_interface::HardwareInterface
{
public:
  /// constructor
  Robot(TiXmlElement *root);

  /// Get a joint state by name
  JointState *getJointState(const std::string &name);

  /// Get the time when the current controller cycle was started
  ros::Time getTime() const
  {
    return hw_.current_time_;
  };

  /// Propagete the joint positions, through the transmissions, to the actuator positions
  void propagateJointPositionToActuatorPosition();

  /// Propagete the actuator efforts, through the transmissions, to the joint efforts
  void propagateActuatorEffortToJointEffort();

  /// Checks if one (or more) of the motors are halted.
  bool isHalted() const;

  /**
   * Each transmission refers to the actuators and joints it connects by name.
   * Since name lookup is slow, for each transmission in the robot model we
   * cache pointers to the actuators and joints that it connects.
   **/
  std::vector<std::vector<ros_ethercat_hardware_interface::Actuator*> > transmissions_in_;

  /**
   * Each transmission refers to the actuators and joints it connects by name.
   * Since name lookup is slow, for each transmission in the robot model we
   * cache pointers to the actuators and joints that it connects.
   **/
  std::vector<std::vector<JointState*> > transmissions_out_;

  /// The joint states mapped to the joint names
  boost::unordered_map<std::string, JointState> joint_states_;

  /// The kinematic/dynamic model of the robot
  urdf::Model robot_model_;

  /// The transmissions
  std::vector<boost::shared_ptr<Transmission> > transmissions_;

  /// get an actuator pointer based on the actuator name. Returns NULL on failure
  ros_ethercat_hardware_interface::Actuator* getActuator(const std::string &name) const
  {
    return hw_.getActuator(name);
  }

  /// get a transmission pointer based on the transmission name. Returns NULL on failure
  boost::shared_ptr<Transmission> getTransmission(const std::string &name) const;

  /// a pointer to the hardware interface. Only for advanced users
  ros_ethercat_hardware_interface::HardwareInterface hw_;

private:
  pluginlib::ClassLoader<Transmission> transmission_loader_;
};

}

#endif
