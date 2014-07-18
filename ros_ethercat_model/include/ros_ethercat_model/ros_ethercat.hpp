/*
 * ros_ethercat.hpp
 *
 *  Created on: 7 Jan 2014
 *      Author: Manos Nikolaidis
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Shadow Robot Company Ltd.
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

#ifndef ROS_ETHERCAT_MODEL_ROS_ETHERCAT_HPP_
#define ROS_ETHERCAT_MODEL_ROS_ETHERCAT_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include "ros_ethercat_model/robot_state.hpp"
#include "ros_ethercat_model/mech_stats_publisher.hpp"
#include "ros_ethercat_hardware/ethercat_hardware.h"


/** \brief Contains robot state information and init, read, write function.
 *
 * The robot state is contained in ros_ethercat_model::RobotStateState object
 * as used by pr2_controller object. eNvertheless, the main loop in main.cpp
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

using std::string;
using boost::ptr_unordered_map;
using ros_ethercat_model::JointState;
using ros_ethercat_model::Actuator;
using ros_ethercat_model::Transmission;

static const string name = "ros_ethercat";

class RosEthercat : public hardware_interface::RobotHW
{
public:
  RosEthercat(ros::NodeHandle &nh, const string &eth, bool allow, TiXmlElement* config) :
    cm_node_(nh, "ethercat_controller_manager"),
    model_(config),
    ec_(name, static_cast<hardware_interface::HardwareInterface*> (&model_), eth, allow)
  {
    for (ptr_unordered_map<string, JointState>::iterator it = model_.joint_states_.begin();
         it != model_.joint_states_.end();
         ++it)
    {
      hardware_interface::JointStateHandle jsh(it->first,
                                               &it->second->position_,
                                               &it->second->velocity_,
                                               &it->second->measured_effort_);
      joint_state_interface_.registerHandle(jsh);

      hardware_interface::JointHandle jh(joint_state_interface_.getHandle(it->first),
                                         &it->second->commanded_effort_);
      joint_command_interface_.registerHandle(jh);
      effort_joint_interface_.registerHandle(jh);
      position_joint_interface_.registerHandle(jh);
    }

    if (!model_.joint_states_.empty())
      mech_stats_publisher_.reset(new MechStatsPublisher(nh, model_));

    registerInterface(&model_);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_command_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
  }
  virtual ~RosEthercat()
  {
  }

  /// propagate position actuator -> joint and set commands to zero
  void read()
  {
    ec_.update(false, false);

    model_.current_time_ = ros::Time::now();
    model_.propagateActuatorPositionToJointPosition();

    for (ptr_unordered_map<string, JointState>::iterator it = model_.joint_states_.begin();
         it != model_.joint_states_.end();
         ++it)
    {
      it->second->joint_statistics_.update(it->second);
      it->second->commanded_effort_ = 0;
    }
  }

  /// propagate effort joint -> actuator and enforce safety limits
  void write()
  {
    /// Modify the commanded_effort_ of each joint state so that the joint limits are satisfied
    for (ptr_unordered_map<string, JointState>::iterator it = model_.joint_states_.begin();
         it != model_.joint_states_.end();
         ++it)
    {
      it->second->enforceLimits();
    }

    model_.propagateJointEffortToActuatorEffort();
    if (!model_.joint_states_.empty())
      mech_stats_publisher_->publish();
  }

  /// stop all actuators
  void shutdown()
  {
    for (boost::ptr_vector<Transmission>::iterator it = model_.transmissions_.begin();
         it != model_.transmissions_.end();
         ++it)
    {
      it->actuator_->command_.enable_ = false;
      it->actuator_->command_.effort_ = 0;
    }
  }

  ros::NodeHandle cm_node_;
  ros_ethercat_model::RobotState model_;
  EthercatHardware ec_;
  boost::scoped_ptr<MechStatsPublisher> mech_stats_publisher_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::JointCommandInterface joint_command_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
};

#endif
