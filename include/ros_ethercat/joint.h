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
 *
 */
#ifndef JOINT_H
#define JOINT_H

#include <tinyxml.h>
#include <urdf_model/joint.h>


namespace ros_ethercat_mechanism_model {

class JointState;

class JointStatistics
{
 public:
  JointStatistics():odometer_(0.0), min_position_(0), max_position_(0),
                    max_abs_velocity_(0.0), max_abs_effort_(0.0),
                    violated_limits_(false), initialized_(false){}

  void update(JointState* s);
  void reset();

  double odometer_;
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
  void enforceLimits();

  /// Returns the safety effort limits given the current position and velocity.
  void getLimits(double &effort_low, double &effort_high);

  /// A pointer to the corresponding urdf::Joint from the urdf::Model
  boost::shared_ptr<const urdf::Joint> joint_;

  /// The joint position in radians or meters (read-only variable)
  double position_;

  /// The joint velocity in randians/sec or meters/sec (read-only variable)
  double velocity_;

  /// The measured joint effort in Nm or N (read-only variable)
  double measured_effort_;

  // joint statistics
  JointStatistics joint_statistics_;

  /// The effort the joint should apply in Nm or N (write-to variable)
  double commanded_effort_;

  /// Bool to indicate if the joint has been calibrated or not
  bool calibrated_;

  /// The position of the optical flag that was used to calibrate this joint
  double reference_position_;

  /// Constructor
  JointState() : position_(0.0), velocity_(0.0), measured_effort_(0.0),
    commanded_effort_(0.0), calibrated_(false), reference_position_(0.0){}
};

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


}

#endif /* JOINT_H */
