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

#ifndef ROS_ETHERCAT_MODEL_JOINT_H
#define ROS_ETHERCAT_MODEL_JOINT_H

#include <map>
#include <string>
#include <vector>
#include <cfloat>
#include <tinyxml.h>
#include <urdf_model/joint.h>

namespace ros_ethercat_model
{

enum
{
  JOINT_NONE,
  JOINT_ROTARY,
  JOINT_CONTINUOUS,
  JOINT_PRISMATIC,
  JOINT_FIXED,
  JOINT_PLANAR,
  JOINT_TYPES_MAX
};

class JointState;

class JointStatistics
{
public:
  JointStatistics() :
    min_position_(0), max_position_(0),
    max_abs_velocity_(0.0), max_abs_effort_(0.0),
    violated_limits_(false), initialized_(false)
  {
  }

  void update(JointState *jnt);
  void reset()
  {
    double tmp = min_position_;
    min_position_ = max_position_;
    max_position_ = tmp;
    max_abs_velocity_ = 0.0;
    max_abs_effort_ = 0.0;
    violated_limits_ = false;
  }

  double min_position_, max_position_;
  double max_abs_velocity_;
  double max_abs_effort_;
  bool violated_limits_;

private:
  bool initialized_;
  double old_position_;
};

class JointState
{
public:
  /// Modify the commanded_effort_ of the joint state so that the joint limits are satisfied
  void enforceLimits()
  {
    double effort_high, effort_low;

    getLimits(effort_low, effort_high);

    // limit the commanded effort based on position, velocity and effort limits
    commanded_effort_ = std::min(std::max(commanded_effort_, effort_low), effort_high);
  }

  /// Returns the safety effort limits given the current position and velocity.
  void getLimits(double &effort_low, double &effort_high)
  {
    // only enforce joints that specify joint limits and safety code
    if (!joint_->safety || !joint_->limits)
    {
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
      vel_high = std::max(-joint_->limits->velocity,
                          std::min(joint_->limits->velocity,
                                   -joint_->safety->k_position * (position_ - joint_->safety->soft_upper_limit)));
      vel_low = std::min(joint_->limits->velocity,
                         std::max(-joint_->limits->velocity,
                                  -joint_->safety->k_position * (position_ - joint_->safety->soft_lower_limit)));
    }

    // Computes the effort bounds based on the velocity and effort bounds.
    effort_high = std::max(-joint_->limits->effort,
                           std::min(joint_->limits->effort,
                                    -joint_->safety->k_velocity * (velocity_ - vel_high)));
    effort_low = std::min(joint_->limits->effort,
                          std::max(-joint_->limits->effort,
                                   -joint_->safety->k_velocity * (velocity_ - vel_low)));
  }

  /// A pointer to the corresponding urdf::Joint from the urdf::Model
  boost::shared_ptr<const urdf::Joint> joint_;

  /// The joint position in radians or meters (read-only variable)
  double position_;

  /// The joint velocity in radians/sec or meters/sec (read-only variable)
  double velocity_;

  /// The joint effort in Nm or N (read-only variable)
  double effort_;

  // joint statistics
  JointStatistics joint_statistics_;

  /// The position the joint should move to in radians or meters (write-to variable)
  double commanded_position_;

  /// The velocity the joint should move with in radians/sec or meters/sec (write-to variable)
  double commanded_velocity_;

  /// The effort the joint should apply in Nm or N (write-to variable)
  double commanded_effort_;

  /// Indicates if the joint has been calibrated or not
  bool calibrated_;

  /// The position of the optical flag that was used to calibrate this joint
  double reference_position_;

  /// Constructor
  JointState() :
    position_(0.0),
    velocity_(0.0),
    effort_(0.0),
    commanded_position_(0.0),
    commanded_velocity_(0.0),
    commanded_effort_(0.0),
    calibrated_(false),
    reference_position_(0.0)
  {
  }
};
inline void JointStatistics::update(JointState *jnt)
{
  if (initialized_)
  {
    if (jnt->joint_->safety && jnt->joint_->limits
        && (fabs(jnt->commanded_effort_) > fabs(jnt->effort_)))
      violated_limits_ = true;
    min_position_ = fmin(jnt->position_, min_position_);
    max_position_ = fmax(jnt->position_, max_position_);
    max_abs_velocity_ = fmax(fabs(jnt->velocity_), max_abs_velocity_);
    max_abs_effort_ = fmax(fabs(jnt->effort_), max_abs_effort_);
  }
  else
  {
    min_position_ = jnt->position_;
    max_position_ = jnt->position_;
    initialized_ = true;
  }
  old_position_ = jnt->position_;
}

}

#endif /* JOINT_H */
