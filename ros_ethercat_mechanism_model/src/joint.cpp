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

#include <ros_ethercat_mechanism_model/joint.hpp>
#include <map>
#include <string>
#include <vector>
#include <cfloat>

using namespace std;
using namespace ros_ethercat_mechanism_model;


void JointStatistics::update(JointState* jnt)
{
  if (initialized_){
    odometer_ += fabs(old_position_ - jnt->position_);
    if (jnt->joint_->safety && jnt->joint_->limits && (fabs(jnt->commanded_effort_) > fabs(jnt->measured_effort_)))
      violated_limits_ = true;
    min_position_ = fmin(jnt->position_, min_position_);
    max_position_ = fmax(jnt->position_, max_position_);
    max_abs_velocity_ = fmax(fabs(jnt->velocity_), max_abs_velocity_);
    max_abs_effort_ = fmax(fabs(jnt->measured_effort_), max_abs_effort_);
  }
  else{
    min_position_ = jnt->position_;
    max_position_ = jnt->position_;
    initialized_ = true;
  }
  old_position_ = jnt->position_;
}


void JointStatistics::reset()
{
  double tmp = min_position_;
  min_position_ = max_position_;
  max_position_ = tmp;
  max_abs_velocity_ = 0.0;
  max_abs_effort_ = 0.0;
  violated_limits_ = false;
}


void JointState::enforceLimits()
{
  double effort_high, effort_low;

  getLimits(effort_low, effort_high);

  // limit the commanded effort based on position, velocity and effort limits
  commanded_effort_ =
    min( max(commanded_effort_, effort_low), effort_high);
}

void JointState::getLimits(double &effort_low, double &effort_high)
{
  // only enforce joints that specify joint limits and safety code
  if (!joint_->safety || !joint_->limits) {
    effort_low = -std::numeric_limits<double>::max();
    effort_high = std::numeric_limits<double>::max();
    return;
  }

  double vel_high = joint_->limits->velocity;
  double vel_low = -joint_->limits->velocity;
  effort_high = joint_->limits->effort;
  effort_low = -joint_->limits->effort;

  // enforce position bounds on rotary and prismatic joints that are calibrated
  if (calibrated_ && (joint_->type == urdf::Joint::REVOLUTE || joint_->type == urdf::Joint::PRISMATIC))
  {
    // Computes the velocity bounds based on the absolute limit and the
    // proximity to the joint limit.
    vel_high = max(-joint_->limits->velocity,
                   min(joint_->limits->velocity,
                       -joint_->safety->k_position * (position_ - joint_->safety->soft_upper_limit)));
    vel_low = min(joint_->limits->velocity,
                  max(-joint_->limits->velocity,
                      -joint_->safety->k_position * (position_ - joint_->safety->soft_lower_limit)));
  }

  // Computes the effort bounds based on the velocity and effort bounds.
  effort_high = max(-joint_->limits->effort,
                    min(joint_->limits->effort,
                        -joint_->safety->k_velocity * (velocity_ - vel_high)));
  effort_low = min(joint_->limits->effort,
                   max(-joint_->limits->effort,
                       -joint_->safety->k_velocity * (velocity_ - vel_low)));
}


