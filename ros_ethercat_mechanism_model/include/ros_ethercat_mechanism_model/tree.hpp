/*
 * Copyright (c) 2011, PAL Robotics S.L.
 * All rights reserved.
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Author: Marcus Liebhardt
 *
 * This class has been derived from the chain class in the
 * ros_ethercat_mechanism_model package in the pr2_mechanism stack for ROS
 * written by Stuart Glaser and Wim Meeussen.
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

#ifndef ros_ethercat_mechanism_model_TREE_H
#define ros_ethercat_mechanism_model_TREE_H

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include "ros_ethercat_mechanism_model/robot.hpp"

namespace ros_ethercat_mechanism_model
{

class Tree
{
public:
  Tree() : kdl_tree_() {};
  ~Tree() {};

  /**
   * \brief initializes the tree object
   * The initializer's most important functionality is to create a vector of joints.
   * This vector is ordered according to the joint's number given by KDL's tree class, which is used by the
   * kdl_parser to create a KDL::Tree from the robot description. This structure is what a KDL tree solver expects.
   * The vector of joints can later be used to read the joints' positions or to send efforts to them.
   *
   * \param robot_state the robot state object containing the robot model and the state of each joint in the robot
   */
  bool init(RobotState *robot_state);

  /// get the position of the joints of the tree as a KDL::JntArray
  void getPositions(KDL::JntArray&) const;

  /// get the position of the joints of the tree as any type with size() and [] lookup
  template <class Vec>
  void getPositions(Vec&) const;

  /// get the velocities of the joints of the tree as a KDL::JntArrayVel (fills in the positions, too)
  void getVelocities(KDL::JntArrayVel&) const;

  /// get the velocities of the joints of the tree as any type with size() and [] lookup
  template <class Vec>
  void getVelocities(Vec&) const;

  /// get the measured joint efforts of the tree's joints as a KDL::JntArray
  void getEfforts(KDL::JntArray&) const;

  /// get the measured joint efforts of the tree's joints as any type with size() and [] lookup
  template <class Vec>
  void getEfforts(Vec&) const;

  /// set the commanded joint efforts of the tree's joints from a KDL::JntArray
  void setEfforts(const KDL::JntArray&);

  /// set the commanded joint efforts of the tree's joints from any type that implements size() and [] lookup
  template <class Vec>
  void setEfforts(const Vec&);

  /// add to the commanded joint efforts of the tree's joints from a KDL::JntArray
  void addEfforts(const KDL::JntArray&);

  /// add to the commanded joint efforts of the tree's joints from any type that implements size() and [] lookup
  template <class Vec>
  void addEfforts(const Vec&);

  /// returns true, if all the joints in the tree are calibrated
  bool allCalibrated() const;

  /// get a KDL::Tree object that respresents the tree
  void toKdl(KDL::Tree&) const;

  /// returns a pointer to the joint state of a joint in the list of the tree's actuated joints (index starts at 0)
  JointState* getJoint(unsigned int) const;

  /// returns the number of actuated joints in the tree
  int size() const;

private:
  KDL::Tree kdl_tree_;
  /// a vector of pointers to joint states; includes only the ones that can be actuated (not fixed joints)
  std::vector<JointState*> joints_;
};

inline void Tree::getPositions(KDL::JntArray& positions) const
{
  assert(positions.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    positions(i) = joints_[i]->position_;
}

template <class Vec>
inline void Tree::getPositions(Vec &v) const
{
  assert((int)v.size() == (int)joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    v[i] = joints_[i]->position_;
}

inline void Tree::getVelocities(KDL::JntArrayVel& velocities) const
{
  assert(velocities.q.rows() == joints_.size());
  assert(velocities.qdot.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    velocities.q(i) = joints_[i]->position_;
    velocities.qdot(i) = joints_[i]->velocity_;
  }
}

template <class Vec>
inline void Tree::getVelocities(Vec &v) const
{
  assert((int)v.size() == (int)joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    v[i] = joints_[i]->velocity_;
}

inline void Tree::getEfforts(KDL::JntArray& efforts) const
{
  assert(efforts.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    efforts(i) = joints_[i]->measured_effort_;
}

template <class Vec>
inline void Tree::getEfforts(Vec &v) const
{
  assert((int)v.size() == (int)joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    v[i] = joints_[i]->measured_effort_;
}

inline void Tree::setEfforts(const KDL::JntArray& efforts)
{
  assert(efforts.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ = efforts(i);
}

template <class Vec>
inline void Tree::setEfforts(const Vec& v)
{
  assert((int)v.size() == (int)joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ = v[i];
}

inline void Tree::addEfforts(const KDL::JntArray& efforts)
{
  assert(efforts.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ += efforts(i);
}

template <class Vec>
inline void Tree::addEfforts(const Vec& v)
{
  assert((int)v.size() == (int)joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ += v[i];
}

inline bool Tree::allCalibrated() const
{
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
      return false;
  }
  return true;
}

inline void Tree::toKdl(KDL::Tree &tree) const
{
  tree = kdl_tree_;
}

inline JointState* Tree::getJoint(unsigned int actuated_joint_i) const
{
  if (actuated_joint_i >= joints_.size())
    return NULL;
  else
    return joints_[actuated_joint_i];
}

inline int Tree::size() const
{
  return joints_.size();
}

} // namespace

#endif /* ros_ethercat_mechanism_model_TREE_H */
