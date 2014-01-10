 /*
 * ros_ethercat.cpp
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

#include "ros_ethercat/ros_ethercat.hpp"
#include <algorithm>
#include <sstream>
#include "ros/console.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;
using namespace pr2_controller_interface;
using namespace boost::accumulators;
using namespace ros;

bool ros_ethercat::initXml(TiXmlElement* config)
{
  if (!model_.initXml(config))
  {
    ROS_ERROR("Failed to initialize pr2 mechanism model");
    return false;
  }
  state_ = new pr2_mechanism_model::RobotState(&model_);
  motors_previously_halted_ = state_->isHalted();

  // pre-allocate for realtime publishing
  pub_mech_stats_.msg_.controller_statistics.resize(0);
  pub_mech_stats_.msg_.actuator_statistics.resize(model_.hw_->actuators_.size());
  int joints_size = 0;
  for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
  {
    int type = state_->joint_states_[i].joint_->type;
    if (type != urdf::Joint::REVOLUTE &&
        type != urdf::Joint::CONTINUOUS &&
        type != urdf::Joint::PRISMATIC)
    {
      ROS_ERROR("Joint state contains joint '%s' of unknown type", state_->joint_states_[i].joint_->name.c_str());
      return false;
    }
    ++joints_size;
  }
  pub_mech_stats_.msg_.joint_statistics.resize(joints_size);

  // get the publish rate for mechanism state
  double publish_rate_joint_state, publish_rate_mechanism_stats;
  cm_node_.param("mechanism_statistics_publish_rate", publish_rate_mechanism_stats, 1.0);
  publish_period_mechanism_stats_ = Duration(1.0/fmax(0.000001, publish_rate_mechanism_stats));

  this->registerInterface(state_);
  return true;
}

void ros_ethercat::publishMechanismStatistics()
{
  ros::Time now = ros::Time::now();
  if (now > last_published_mechanism_stats_ + publish_period_mechanism_stats_)
  {
    if (pub_mech_stats_.trylock())
    {
      while (last_published_mechanism_stats_ + publish_period_mechanism_stats_ < now)
        last_published_mechanism_stats_ = last_published_mechanism_stats_ + publish_period_mechanism_stats_;

      // joint state
      unsigned int j = 0;
      for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
      {
        int type = state_->joint_states_[i].joint_->type;
        if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
        {
          assert(j < pub_mech_stats_.msg_.joint_statistics.size());
          pr2_mechanism_model::JointState *in = &state_->joint_states_[i];
          pr2_mechanism_msgs::JointStatistics *out = &pub_mech_stats_.msg_.joint_statistics[j];
          out->timestamp = now;
          out->name = state_->joint_states_[i].joint_->name;
          out->position = in->position_;
          out->velocity = in->velocity_;
          out->measured_effort = in->measured_effort_;
          out->commanded_effort = in->commanded_effort_;
          out->is_calibrated = in->calibrated_;
          out->violated_limits = in->joint_statistics_.violated_limits_;
          out->odometer = in->joint_statistics_.odometer_;
          out->min_position = in->joint_statistics_.min_position_;
          out->max_position = in->joint_statistics_.max_position_;
          out->max_abs_velocity = in->joint_statistics_.max_abs_velocity_;
          out->max_abs_effort = in->joint_statistics_.max_abs_effort_;
          in->joint_statistics_.reset();
          ++j;
        }
      }

      // actuator state
      unsigned int i = 0;
      for (ActuatorMap::const_iterator it = model_.hw_->actuators_.begin(); it != model_.hw_->actuators_.end(); ++i, ++it)
      {
        pr2_mechanism_msgs::ActuatorStatistics *out = &pub_mech_stats_.msg_.actuator_statistics[i];
        ActuatorState *in = &(it->second->state_);
        out->timestamp = now;
        out->name = it->first;
        out->encoder_count = in->encoder_count_;
        out->encoder_offset = in->zero_offset_;
        out->position = in->position_;
        out->timestamp = ros::Time(in->timestamp_);
        out->device_id = in->device_id_;
        out->encoder_velocity = in->encoder_velocity_;
        out->velocity = in->velocity_;
        out->calibration_reading = in->calibration_reading_;
        out->calibration_rising_edge_valid = in->calibration_rising_edge_valid_;
        out->calibration_falling_edge_valid = in->calibration_falling_edge_valid_;
        out->last_calibration_rising_edge = in->last_calibration_rising_edge_;
        out->last_calibration_falling_edge = in->last_calibration_falling_edge_;
        out->is_enabled = in->is_enabled_;
        out->halted = in->halted_;
        out->last_commanded_current = in->last_commanded_current_;
        out->last_executed_current = in->last_executed_current_;
        out->last_measured_current = in->last_measured_current_;
        out->last_commanded_effort = in->last_commanded_effort_;
        out->last_executed_effort = in->last_executed_effort_;
        out->last_measured_effort = in->last_measured_effort_;
        out->motor_voltage = in->motor_voltage_;
        out->num_encoder_errors = in->num_encoder_errors_;
      }

      // controller state
      std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
      for (unsigned int i = 0; i < controllers.size(); ++i)
      {
        pr2_mechanism_msgs::ControllerStatistics *out = &pub_mech_stats_.msg_.controller_statistics[i];
        out->timestamp = now;
        out->running = controllers[i].c->isRunning();
        out->max_time = ros::Duration(extract::max(controllers[i].stats->acc));
        out->mean_time = ros::Duration(extract::mean(controllers[i].stats->acc));
        out->variance_time = ros::Duration(sqrt((double) extract::variance(controllers[i].stats->acc)));
        out->num_control_loop_overruns = controllers[i].stats->num_control_loop_overruns;
        out->time_last_control_loop_overrun = controllers[i].stats->time_last_control_loop_overrun;
      }

      pub_mech_stats_.msg_.header.stamp = ros::Time::now();
      pub_mech_stats_.unlockAndPublish();
    }
  }
}

