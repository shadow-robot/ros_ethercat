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

// Author: Stuart Glaser

#ifndef MECHANISM_MODEL_CHAIN_H
#define MECHANISM_MODEL_CHAIN_H

#include "ros_ethercat_mechanism_model/robot.hpp"
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>

namespace ros_ethercat_mechanism_model {

class Chain
{
public:
  Chain() {}
  ~Chain() {}

  /** \brief initialize the chain object
   *
   * \param robot_state the robot state object containing the robot model and the state of each joint in the robot
   * \param root the name of the root link of the chain
   * \param tip the name of the tip link of the chain
   *
   */
  bool init(Robot *robot_state, const std::string &root, const std::string &tip);

  /// get the joint positions of the chain as a std vector
  void getPositions(std::vector<double>&);
  /// get the joint positions of the chain as a kdl jnt array
  void getPositions(KDL::JntArray&);
  /// gets the joint positions of the chain as any type with size() and []
  template <class Vec>
  void getPositions(Vec &v)
  {
    assert((int)v.size() == (int)joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->position_;
  }

  /// get the joint velocities of the chain as a std vector
  void getVelocities(std::vector<double>&);
  /// get the joint velocities and positoin of the chain as a kdl jnt array vel.  Fills in the positions too.
  void getVelocities(KDL::JntArrayVel&);
  /// gets the joint velocities of the chain as any type with size() and []
  template <class Vec>
  void getVelocities(Vec &v)
  {
    assert((int)v.size() == (int)joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->velocity_;
  }


  /// get the measured joint efforts of the chain as a std vector
  void getEfforts(std::vector<double>&);
  /// get the measured joint efforts of the chain as a kdl jnt array
  void getEfforts(KDL::JntArray&);

  /// set the commanded joint efforts of the chain as a std vector
  void setEfforts(KDL::JntArray&);
  /// set the commanded joint efforts of the chain as a kdl jnt array
  void addEfforts(KDL::JntArray&);

  /*!
   * \brief Adds efforts from any type that implements size() and [] lookup.
   */
  template <class Vec>
  void addEfforts(const Vec& v)
  {
    assert((int)v.size() == (int)joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ += v[i];
  }

  /// returns true if all the joints in the chain are calibrated
  bool allCalibrated();

  /// get a kdl chain object that respresents the chain from root to tip
  void toKDL(KDL::Chain &chain);

  /** \brief get a joint state of an actuated joint of the chain.
   *
   * the actuated_joint_i index starts at zero
   * fixed joints are ignored in the list of actuated joints
   */
  JointState* getJoint(unsigned int actuated_joint_i);

  /// Returns the number of actuated joints in the chain
  int size() const { return joints_.size(); }

private:
  ros_ethercat_mechanism_model::Robot *robot_state_;
  KDL::Chain kdl_chain_;

  std::vector< JointState* > joints_;  // ONLY joints that can be actuated (not fixed joints)
};

}

#endif
