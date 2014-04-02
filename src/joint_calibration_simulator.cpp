/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Wim Meeussen

#include "ros_ethercat/joint_calibration_simulator.h"
#include <angles/angles.h>


namespace ros_ethercat_mechanism_model {


JointCalibrationSimulator::JointCalibrationSimulator()
  : calibration_initialized_(false),
    calibration_has_rising_(false), 
    calibration_has_falling_(false),
    calibration_continuous_(false),
    got_info_(false)
{
}

void JointCalibrationSimulator::GetJointCalibrationInfo(ros_ethercat_mechanism_model::JointState* js)
{
  assert(js->joint_);

  // simulate calibration backward propagation
  if (js->joint_->calibration){
    if (js->joint_->calibration->rising){
      calibration_has_rising_ = true;
      calibration_rising_ = *(js->joint_->calibration->rising);
    }
    if (js->joint_->calibration->falling){
      calibration_has_falling_ = true;
      calibration_falling_ = *(js->joint_->calibration->falling);
    }
  }
  

  // continuous joints
  if (js->joint_->type == urdf::Joint::CONTINUOUS){
    calibration_continuous_ = true;
    if (calibration_has_rising_ && !calibration_has_falling_){
      calibration_has_falling_ = true;
      calibration_falling_ = calibration_rising_ + M_PI;
    }
    if (!calibration_has_rising_ && calibration_has_falling_){
      calibration_has_rising_ = true;
      calibration_rising_ = calibration_falling_ + M_PI;
    }
    calibration_rising_ = angles::normalize_angle(calibration_rising_);
    calibration_falling_ = angles::normalize_angle(calibration_falling_);
    if (calibration_rising_ < calibration_falling_)
      calibration_bump_ = true;
    else
      calibration_bump_ = false;
  }

  // check
  if (js->joint_->type != urdf::Joint::CONTINUOUS && calibration_has_rising_ && calibration_has_falling_)
      ROS_ERROR("Non-continuous joint with both rising and falling edge not supported");

  this->got_info_ = true;
}

void JointCalibrationSimulator::simulateJointCalibration(ros_ethercat_mechanism_model::JointState* js, ros_ethercat_hardware_interface::Actuator* as)
{
  // setup calibration information for the joint
  if (!this->got_info_) this->GetJointCalibrationInfo(js);

  // current joint_angle
  double pos = js->position_ - js->reference_position_;
  double as_pos = as->state_.position_;

  // compute calibration reading
  if (calibration_continuous_){
    double pos_c = angles::normalize_angle(pos);
    if (calibration_bump_){
      if (pos_c > calibration_rising_ && pos_c < calibration_falling_)
        as->state_.calibration_reading_ = true;
      else
        as->state_.calibration_reading_ = false;
    }
    else{
      if (pos_c < calibration_rising_ && pos_c > calibration_falling_)
        as->state_.calibration_reading_ = false; // in low part
      else
        as->state_.calibration_reading_ = true;
    }
  }
  else{
    if (calibration_has_rising_)
      as->state_.calibration_reading_ = pos > calibration_rising_;
    else if (calibration_has_falling_)
      as->state_.calibration_reading_ = pos < calibration_falling_;
  }

  if (calibration_initialized_){
    // tripped calibration flag
    //ROS_ERROR("debug: %s %d %d",js->joint_->name.c_str(),old_calibration_reading_ ,as->state_.calibration_reading_);
    if (old_calibration_reading_ != as->state_.calibration_reading_){
      if (as->state_.calibration_reading_){ // low to high
        if (pos > old_calibration_pos_) // joint pos increasing and we are in the high region
        {
          as->state_.calibration_rising_edge_valid_ = true;
          as->state_.last_calibration_rising_edge_ = old_calibration_as_pos_;
        }
        else
        {
          as->state_.calibration_falling_edge_valid_ = true;
          as->state_.last_calibration_falling_edge_ = old_calibration_as_pos_;
        }
      }
      else{ // high to low
        if (pos > old_calibration_pos_) // joint pos increasing and we are in the low region
        {
          as->state_.calibration_falling_edge_valid_ = true;
          as->state_.last_calibration_falling_edge_ = old_calibration_as_pos_;
        }
        else
        {
          as->state_.calibration_rising_edge_valid_ = true;
          as->state_.last_calibration_rising_edge_ = old_calibration_as_pos_;
        }
      }
    }
  }

  // store state
  old_calibration_reading_ = as->state_.calibration_reading_;
  old_calibration_pos_ = pos;
  old_calibration_as_pos_ = as_pos;
  calibration_initialized_ = true;
}



} //namespace

