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
 * ros_ethercat_model package in the pr2_mechanism stack for ROS
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

#ifndef ROS_ETHERCAT_MODEL_TREE_H
#define ROS_ETHERCAT_MODEL_TREE_H

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include "ros_ethercat_model/robot_state.hpp"
#include <kdl_parser/kdl_parser.hpp>

namespace ros_ethercat_model
{

class Tree
{
public:
  Tree() :
    kdl_tree_()
  {
  }
  /**
   * \brief initializes the tree object
   * The initializer's most important functionality is to create a vector of joints.
   * This vector is ordered according to the joint's number given by KDL's tree class, which is used by the
   * kdl_parser to create a KDL::Tree from the robot description. This structure is what a KDL tree solver expects.
   * The vector of joints can later be used to read the joints' positions or to send efforts to them.
   *
   * \param robot_state the robot state object containing the robot model and the state of each joint in the robot
   */
  bool init(RobotState *robot_state)
  {
    KDL::SegmentMap segmentMap;

    // construct the kdl tree
    if (!kdl_parser::treeFromUrdfModel(robot_state->robot_model_, kdl_tree_))
    {
      ROS_ERROR("Failed to construct KDL:Tree from robot_state's URDF model! Aborting ...");
    }
    else
    {
      ROS_INFO("KDL::Tree successful created.");

      segmentMap = kdl_tree_.getSegments();
    }

    // the first step of extracting the joints from the tree is to go through all tree_elements, check for a joint,
    // and check in case a joint is found, if it is not of not of type KDL::Joint::None

    // map for saving the temporary result of the joint extraction from the tree
    std::map<unsigned int, std::string> jointMap;

    ROS_DEBUG("Extracting all joints from the tree, which are not of type KDL::Joint::None.");
    for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end();
         ++seg_it)
    {
      if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
        jointMap[seg_it->second.q_nr] = seg_it->second.segment.getJoint().getName().c_str();
    }

    // in the second step the joints found get checked, if they appear in the JointState vector of the robot
    ROS_DEBUG("Checking, if extracted joints can be found in the JointState vector of the robot.");
    joints_.clear();
    for (std::map<unsigned int, std::string>::const_iterator jnt_it = jointMap.begin();
         jnt_it != jointMap.end(); ++jnt_it)
    {
      JointState* jnt = robot_state->getJointState(jnt_it->second.c_str());
      if (!jnt)
      {
        ROS_ERROR("Joint '%s' has not been found in the robot's joint state vector! Aborting ...",
                  jnt_it->second.c_str());
        return false;
      }
      joints_.push_back(jnt);
    }

    ROS_DEBUG("The result after joint extraction and checking:");
    for (unsigned int i = 0; i < joints_.size(); ++i)
    {
      ROS_DEBUG("joints_[%d]: joint_.name = %s", i, joints_[i]->joint_->name.c_str());
    }

    ROS_INFO("Added %i joints", int(joints_.size()));

    return true;
  }

  /// get the position of the joints of the tree as a KDL::JntArray
  void getPositions(KDL::JntArray& positions) const
  {
    assert(positions.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      positions(i) = joints_[i]->position_;
  }

  /// get the position of the joints of the tree as any type with size() and [] lookup
  template<class Vec>
  void getPositions(Vec &v) const
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->position_;
  }

  /// get the velocities of the joints of the tree as a KDL::JntArrayVel (fills in the positions, too)
  void getVelocities(KDL::JntArrayVel &velocities) const
  {
    assert(velocities.q.rows() == joints_.size());
    assert(velocities.qdot.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
    {
      velocities.q(i) = joints_[i]->position_;
      velocities.qdot(i) = joints_[i]->velocity_;
    }
  }

  /// get the velocities of the joints of the tree as any type with size() and [] lookup
  template<class Vec>
  void getVelocities(Vec &v) const
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->velocity_;
  }

  /// get the measured joint efforts of the tree's joints as a KDL::JntArray
  void getEfforts(KDL::JntArray &efforts) const
  {
    assert(efforts.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      efforts(i) = joints_[i]->effort_;
  }

  /// get the measured joint efforts of the tree's joints as any type with size() and [] lookup
  template<class Vec>
  void getEfforts(Vec &v) const
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      v[i] = joints_[i]->effort_;
  }

  /// set the commanded joint efforts of the tree's joints from a KDL::JntArray
  void setEfforts(const KDL::JntArray &efforts)
  {
    assert(efforts.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ = efforts(i);
  }

  /// set the commanded joint efforts of the tree's joints from any type that implements size() and [] lookup
  template<class Vec>
  void setEfforts(const Vec &v)
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ = v[i];
  }

  /// add to the commanded joint efforts of the tree's joints from a KDL::JntArray
  void addEfforts(const KDL::JntArray &efforts)
  {
    assert(efforts.rows() == joints_.size());
    for (unsigned int i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ += efforts(i);
  }

  /// add to the commanded joint efforts of the tree's joints from any type that implements size() and [] lookup
  template<class Vec>
  void addEfforts(const Vec &v)
  {
    assert((int) v.size() == (int) joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
      joints_[i]->commanded_effort_ += v[i];
  }

  /// returns true, if all the joints in the tree are calibrated
  bool allCalibrated() const
  {
    for (unsigned int i = 0; i < joints_.size(); ++i)
      if (!joints_[i]->calibrated_)
        return false;
    return true;
  }

  /// get a KDL::Tree object that represents the tree
  void toKdl(KDL::Tree &tree) const
  {
    tree = kdl_tree_;
  }

  /// returns a pointer to the joint state of a joint in the list of the tree's actuated joints (index starts at 0)
  JointState* getJoint(unsigned int actuated_joint_i) const
  {
    if (actuated_joint_i >= joints_.size())
      return NULL;
    else
      return joints_[actuated_joint_i];
  }

  /// returns the number of actuated joints in the tree
  int size() const
  {
    return joints_.size();
  }

private:
  KDL::Tree kdl_tree_;
  /// a vector of pointers to joint states; includes only the ones that can be actuated (not fixed joints)
  std::vector<JointState*> joints_;
};

} // namespace

#endif /* ros_ethercat_model_TREE_H */
