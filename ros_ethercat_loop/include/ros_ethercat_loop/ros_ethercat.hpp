/*
 * ros_ethercat.hpp
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

#ifndef SR_ETHERCAT_INTERFACE_HPP_
#define SR_ETHERCAT_INTERFACE_HPP_

#include <ros/ros.h>
#include <tinyxml.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros_ethercat_mechanism_model/robot.hpp>
#include <controller_manager/controller_manager.h>
#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <boost/scoped_ptr.hpp>

/** \brief Contains robot state information and init, read, write function.
 *
 * The robot state is contained in ros_ethercat_mechanism_model::RobotState object
 * as used by pr2_controller object. Nevertheless, the main loop in main.cpp
 * instantiates a ros_control controller_manager. So a pr2_controller with few modifications
 * may be loaded with controller_manager with RobotState as a custom interface.
 *
 * ros_control interfaces are exposed alongside RobotState. So controllers from
 * ros_controllers package may also be loaded. These new interfaces contain pointers
 * to data in RobotState so there is no copying or data redundancy.
 *
 * The read and write functions will call the propagate functions of pr2_transmissions.
 * Hardware read and write takes place in the EthercatHardware object in main.cpp
 *
 * initXml, read and write should be called inside main.cpp
 */

class ros_ethercat : public hardware_interface::RobotHW
{
public:
  ros_ethercat(ros_ethercat_hardware_interface::HardwareInterface *hw, ros::NodeHandle &nh) :
    model_(hw), state_(NULL), cm_node_(nh, "controller_manager")
  {}

  virtual ~ros_ethercat() {}

  /**
   * Initialize Robot and RobotState objects from pointer to xml data and register interfaces.
   * The pr2_transmissions whose propagate functions will be called are
   * also initialized by this function
   *
   */
  bool initXml(TiXmlElement* config);

  /// propagate position actuator -> joint and set commands to zero
  void read();

  /// propagate effort joint -> actuator and enforce safety limits
  void write();

  ros::NodeHandle cm_node_;

  ros_ethercat_mechanism_model::Robot model_;
  boost::scoped_ptr<ros_ethercat_mechanism_model::RobotState> state_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::JointCommandInterface joint_command_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
};

#endif /* SR_ETHERCAT_INTERFACE_HPP_ */
