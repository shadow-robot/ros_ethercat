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

#ifndef ROS_ETHERCAT_MODEL_CHAIN_H
#define ROS_ETHERCAT_MODEL_CHAIN_H

#include "ros_ethercat_model/robot_state.hpp"
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace ros_ethercat_model
{

class Chain
{
public:
  /** \brief initialize the chain object
   *
   * \param robot_state the robot state object containing the robot model and the state of each joint in the robot
   * \param root the name of the root link of the chain
   * \param tip the name of the tip link of the chain
   *
   */
  bool init(RobotState *robot_state, const std::string &root, const std::string &tip)
  {
    robot_state_ = robot_state;
    // Constructs the kdl chain
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_state->robot_model_, kdl_tree))
    {
      ROS_ERROR("Could not convert urdf into kdl tree");
      return false;
    }
    bool res;
    try
    {
      res = kdl_tree.getChain(root, tip, kdl_chain_);
    }
    catch (...)
    {
      res = false;
    }
    if (!res)
    {
      ROS_ERROR("Could not extract chain between %s and %s from kdl tree",
                root.c_str(), tip.c_str());
      return false;
    }
    // Pulls out all the joint indices
    joints_.clear();
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++)
    {
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      {
        JointState* jnt = robot_state->getJointState(kdl_chain_.getSegment(i).getJoint().getName());
        if (!jnt)
        {
          ROS_ERROR("Joint '%s' is not found in joint state vector",
                    kdl_chain_.getSegment(i).getJoint().getName().c_str());
          return false;
        }
        joints_.push_back(jnt);
      }
    }
    ROS_DEBUG("Added %i joints", int(joints_.size()));
    return true;
  }
  void getPositions(std::vector<double> &positions)
  {
    positions.clear();
    for (unsigned int i = 0; i < joints_.size(); ++i)
      positions.push_back(joints_[i]->position_);
  }
  void getPositions(KDL::JntArray &a)
  {
    assert(a.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      a(i) = joints_[i]->position_;
  }

  /// gets the joint positions of the chain as any type with size() and []
  template <class Vec> void getPositions(Vec &v)
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->position_;
  }

  /// get the joint velocities of the chain as a std vector
  void getVelocities(std::vector<double> &velocities)
  {
    velocities.clear();
    for (unsigned int i = 0; i < joints_.size(); ++i)
      velocities.push_back(joints_[i]->velocity_);
  }
  /// get the joint velocities and position of the chain as a kdl jnt array vel.  Fills in the positions too.
  void getVelocities(KDL::JntArrayVel &a)
  {
    assert(a.q.rows() == joints_.size());
    assert(a.qdot.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
    {
      a.q(i) = joints_[i]->position_;
      a.qdot(i) = joints_[i]->velocity_;
    }
  }

  /// gets the joint velocities of the chain as any type with size() and []
  template <class Vec> void getVelocities(Vec &v)
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->velocity_;
  }
  void getEfforts(std::vector<double> &efforts)
  {
    efforts.clear();
    for (unsigned int i = 0; i < joints_.size(); ++i)
      efforts.push_back(joints_[i]->effort_);
  }

  /// get the measured joint efforts of the chain as a kdl jnt array
  void getEfforts(KDL::JntArray &a)
  {
    assert(a.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      a(i) = joints_[i]->effort_;
  }

  /// set the commanded joint efforts of the chain as a std vector
  void setEfforts(KDL::JntArray &a)
  {
    assert(a.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ = a(i);
  }

  /// set the commanded joint efforts of the chain as a kdl jnt array
  void addEfforts(KDL::JntArray &a)
  {
    assert(a.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ += a(i);
  }

  /// Adds efforts from any type that implements size() and [] lookup.
  template <class Vec> void addEfforts(const Vec& v)
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ += v[i];
  }

  /// returns true if all the joints in the chain are calibrated
  bool allCalibrated()
  {
    for (unsigned int i = 0; i < joints_.size(); ++i)
      if (!joints_[i]->calibrated_)
        return false;
    return true;
  }

  /// get a kdl chain object that represents the chain from root to tip
  void toKDL(KDL::Chain &chain)
  {
    chain = kdl_chain_;
  }
  /** \brief get a joint state of an actuated joint of the chain.
   *
   * the actuated_joint_i index starts at zero
   * fixed joints are ignored in the list of actuated joints
   */
  JointState* getJoint(unsigned int actuated_joint_i)
  {
    if (actuated_joint_i >= joints_.size())
      return NULL;
    else
      return joints_[actuated_joint_i];
  }

  /// Returns the number of actuated joints in the chain
  int size() const
  {
    return joints_.size();
  }

private:
  RobotState *robot_state_;
  KDL::Chain kdl_chain_;

  std::vector< JointState* > joints_; // ONLY joints that can be actuated (not fixed joints)
};

}

#endif
