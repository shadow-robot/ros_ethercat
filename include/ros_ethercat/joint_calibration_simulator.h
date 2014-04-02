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
#ifndef JOINT_CALIBRATION_SIMULATOR_H
#define JOINT_CALIBRATION_SIMULATOR_H

#include "ros_ethercat/joint.h"
#include "ros_ethercat/hardware_interface.hpp"
#include <urdf_model/joint.h>


namespace ros_ethercat_mechanism_model
{

class JointCalibrationSimulator
{
public:
  JointCalibrationSimulator();
  void simulateJointCalibration(ros_ethercat_mechanism_model::JointState*,
                                ros_ethercat_hardware_interface::Actuator*);

private:
  void GetJointCalibrationInfo(ros_ethercat_mechanism_model::JointState*);
  bool calibration_initialized_;
  bool calibration_has_rising_, calibration_has_falling_, calibration_continuous_;
  double calibration_rising_, calibration_falling_;
  bool got_info_;
  bool calibration_bump_;
  bool old_calibration_reading_;
  double old_calibration_pos_;
  double old_calibration_as_pos_;
};
}
#endif
