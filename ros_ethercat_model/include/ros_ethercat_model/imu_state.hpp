/*
 * robot_state.hpp
 *
 *  Created on: 23 October 2017
 *      Author: Daniel Greenwald
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Shadow Robot Company Ltd.
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

#ifndef ROS_ETHERCAT_MODEL_IMU_STATE_HPP_
#define ROS_ETHERCAT_MODEL_IMU_STATE_HPP_

#include <string>
#include <hardware_interface/imu_sensor_interface.h>

using std::string;

namespace ros_ethercat_model
{

  class ImuState
  {
  public:
    double orientation_[4];
    double angular_velocity_[3];
    double linear_acceleration_[3];
    double orientation_covariance_[9];
    double angular_velocity_covariance_[9];
    double linear_acceleration_covariance_[9];
    hardware_interface::ImuSensorHandle::Data data_;

    ImuState(string name, string frame_id)
    {
      data_.name = name;
      data_.frame_id = frame_id;
      data_.orientation = orientation_;
      data_.orientation_covariance = orientation_covariance_;
      data_.angular_velocity = angular_velocity_;
      data_.angular_velocity_covariance = angular_velocity_covariance_;
      data_.linear_acceleration = linear_acceleration_;
      data_.linear_acceleration_covariance = linear_acceleration_covariance_;

    };
    ImuState(){};

  };
};

#endif // ROS_ETHERCAT_MODEL_IMU_STATE_HPP_
