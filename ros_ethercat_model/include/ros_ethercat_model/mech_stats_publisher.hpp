/*
 * mech_stats_publisher.hpp
 *
 *  Created on: 15 April 2014
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

#ifndef ROS_ETHERCAT_MODEL_MECH_STATS_PUBLISHER_HPP_
#define ROS_ETHERCAT_MODEL_MECH_STATS_PUBLISHER_HPP_

#include <ros_ethercat_model/robot_state.hpp>
#include <sr_robot_msgs/MechanismStatistics.h>
#include <sr_robot_msgs/JointStatistics.h>
#include <sr_robot_msgs/ActuatorStatistics.h>
#include <realtime_tools/realtime_publisher.h>

using std::string;
using std::vector;
using boost::unordered_map;
using boost::ptr_vector;
using boost::ptr_unordered_map;
using ros::Duration;
using ros::Time;
using ros::NodeHandle;
using realtime_tools::RealtimePublisher;
using sr_robot_msgs::MechanismStatistics;
using sr_robot_msgs::JointStatistics;
using sr_robot_msgs::ActuatorStatistics;
using ros_ethercat_model::JointState;
using ros_ethercat_model::Actuator;
using ros_ethercat_model::ActuatorState;
using ros_ethercat_model::RobotState;
using ros_ethercat_model::Transmission;

class MechStatsPublisher
{
public:
  MechStatsPublisher(NodeHandle &nh, RobotState &state) :
    state_(state),
    pub_mech_stats_(nh, "mechanism_statistics", 1),
    last_published_mechanism_stats_(Time::now())
  {
    pub_mech_stats_.msg_.joint_statistics.resize(state_.joint_states_.size());
    pub_mech_stats_.msg_.actuator_statistics.resize(state_.transmissions_.size());

    double publish_rate_mechanism_stats;
    nh.param("mechanism_statistics_publish_rate", publish_rate_mechanism_stats, 1.0);
    publish_period_mechanism_stats_ = Duration(1.0 / fmax(0.000001, publish_rate_mechanism_stats));
  }
  void publish(const ros::Time &now)
  {
    if (now > last_published_mechanism_stats_ + publish_period_mechanism_stats_)
    {
      if (pub_mech_stats_.trylock())
      {
        while (last_published_mechanism_stats_ + publish_period_mechanism_stats_ < now)
          last_published_mechanism_stats_ = last_published_mechanism_stats_ + publish_period_mechanism_stats_;

        ptr_unordered_map<string, JointState>::iterator jin = state_.joint_states_.begin();
        vector<JointStatistics>::iterator jout = pub_mech_stats_.msg_.joint_statistics.begin();
        while (jin != state_.joint_states_.end() &&
               jout != pub_mech_stats_.msg_.joint_statistics.end())
        {
          int type = jin->second->joint_->type;
          if (type != urdf::Joint::REVOLUTE && type != urdf::Joint::CONTINUOUS && type != urdf::Joint::PRISMATIC)
            continue;
          jout->timestamp = now;
          jout->name = jin->second->joint_->name;
          jout->position = jin->second->position_;
          jout->velocity = jin->second->velocity_;
          jout->measured_effort = jin->second->effort_;
          jout->commanded_effort = jin->second->commanded_effort_;
          jout->is_calibrated = jin->second->calibrated_;
          jout->violated_limits = jin->second->joint_statistics_.violated_limits_;
          jout->min_position = jin->second->joint_statistics_.min_position_;
          jout->max_position = jin->second->joint_statistics_.max_position_;
          jout->max_abs_velocity = jin->second->joint_statistics_.max_abs_velocity_;
          jout->max_abs_effort = jin->second->joint_statistics_.max_abs_effort_;
          jin->second->joint_statistics_.reset();
          ++jin;
          ++jout;
        }

        ptr_vector<Transmission>::iterator tin = state_.transmissions_.begin();
        vector<ActuatorStatistics>::iterator aout = pub_mech_stats_.msg_.actuator_statistics.begin();
        while (tin != state_.transmissions_.end() &&
               aout != pub_mech_stats_.msg_.actuator_statistics.end())
        {
          aout->timestamp = now;
          aout->name = tin->actuator_->name_;
          aout->position = tin->actuator_->state_.position_;
          aout->device_id = tin->actuator_->state_.device_id_;
          aout->velocity = tin->actuator_->state_.velocity_;
          aout->is_enabled = true;
          aout->last_commanded_current = tin->actuator_->state_.last_commanded_current_;
          aout->last_measured_current = tin->actuator_->state_.last_measured_current_;
          aout->last_commanded_effort = tin->actuator_->state_.last_commanded_effort_;
          aout->last_measured_effort = tin->actuator_->state_.last_measured_effort_;
          aout->motor_voltage = tin->actuator_->state_.motor_voltage_;
          ++tin;
          ++aout;
        }

        pub_mech_stats_.msg_.header.stamp = Time::now();
        pub_mech_stats_.unlockAndPublish();
      }
    }
  }

  RobotState &state_;
  RealtimePublisher<MechanismStatistics> pub_mech_stats_;
  Duration publish_period_mechanism_stats_;
  Time last_published_mechanism_stats_;
};
#endif
