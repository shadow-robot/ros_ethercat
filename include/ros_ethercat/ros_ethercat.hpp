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

#include <string>
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <pr2_mechanism_model/robot.h>
#include "ros_ethercat/controller_spec.h"
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <controller_manager/controller_manager.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <std_srvs/Empty.h>

class ros_ethercat : public hardware_interface::RobotHW
{
public:
  ros_ethercat(pr2_hardware_interface::HardwareInterface *hw, ros::NodeHandle nh) :
    model_(hw),
    state_(NULL),
    cm_node_(nh, "controller_manager"),
    pub_joint_state_(nh, "joint_states", 1),
    pub_mech_stats_(nh, "mechanism_statistics", 1),
    last_published_joint_state_(ros::Time::now()),
    last_published_mechanism_stats_(ros::Time::now())
  {}

  virtual ~ros_ethercat()
  {
    if (state_)
      delete state_;
  }

  bool configure();
  bool initXml(TiXmlElement* config);

  pr2_mechanism_model::Robot model_;
  pr2_mechanism_model::RobotState *state_;

private:
  ros::NodeHandle cm_node_;

  // for controller statistics
  Statistics pre_update_stats_;
  Statistics update_stats_;
  Statistics post_update_stats_;

  // for publishing constroller state
  void publishJointState();
  void publishMechanismStatistics();
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state_;
  realtime_tools::RealtimePublisher<pr2_mechanism_msgs::MechanismStatistics> pub_mech_stats_;
  ros::Duration publish_period_joint_state_, publish_period_mechanism_stats_;
  ros::Time last_published_joint_state_, last_published_mechanism_stats_;

  int current_controllers_list_, used_by_realtime_;
  std::vector<ControllerSpec> controllers_lists_[2];

  bool motors_previously_halted_;
};

#endif /* SR_ETHERCAT_INTERFACE_HPP_ */
