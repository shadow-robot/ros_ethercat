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
#include <boost/ptr_container/ptr_unordered_map.hpp>
#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_mechanism_model/joint.hpp"
#include "ros_ethercat_mechanism_model/transmission.hpp"
#include <ros_ethercat_mechanism_model/hardware_interface.hpp>

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
 * should apply. Controllers can get access to the hard realtime clock through current_time_
 *
 * Some specialized controllers (such as the calibration controllers) can get access
 * to actuator states, and transmission states.
 *
 * Devices with Digital, Analogue and PWM I/O can use the GeneralIOs
 */
class Robot : public hardware_interface::HardwareInterface
{
public:
  /// constructor
  Robot(TiXmlElement *root);

  /// Propagete the joint positions, through the transmissions, to the actuator positions
  void propagateJointPositionToActuatorPosition();

  /// Propagete the actuator efforts, through the transmissions, to the joint efforts
  void propagateActuatorEffortToJointEffort();

  /// get an actuator pointer based on the actuator name. Returns NULL on failure
  Actuator* getActuator(const std::string &name) const;

  /// get a transmission pointer based on the transmission name. Returns NULL on failure
  Transmission* getTransmission(const std::string &name) const;

  /*! \brief Get a pointer to the Custom Hardware device by name
   *
   *  \param name The name of the Custom Hardware device
   *  \return A pointer to a CustomHW.  Returns NULL if name is not valid.
   */
  CustomHW* getCustomHW(const std::string &name) const;

  /// Get a joint state by name
  JointState* getJointState(const std::string &name) const;

  pluginlib::ClassLoader<Transmission> transmission_loader_;

  ros::Time current_time_; //!< The time at which the commands were sent to the hardware

  /**
   * Each transmission refers to the actuators and joints it connects by name.
   * Since name lookup is slow, for each transmission in the robot model we
   * cache pointers to the actuators and joints that it connects.
   **/
  std::vector<std::vector<Actuator*> > transmissions_in_;

  /**
   * Each transmission refers to the actuators and joints it connects by name.
   * Since name lookup is slow, for each transmission in the robot model we
   * cache pointers to the actuators and joints that it connects.
   **/
  std::vector<std::vector<JointState*> > transmissions_out_;

  /// The joint states mapped to the joint names
  boost::unordered_map<std::string, JointState> joint_states_;

  /// The actuators mapped to their names
  boost::ptr_unordered_map<std::string, Actuator> actuators_;

  /// GeneralIO structures mapped to their names
  boost::ptr_unordered_map<std::string, CustomHW> custom_hws_;

  /// The kinematic/dynamic model of the robot
  urdf::Model robot_model_;

  /// The transmissions
  std::vector<Transmission> transmissions_;
};

}

#endif
