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

// Author: Stuart Glaser, Wim Meeussen

#include "ros_ethercat_mechanism_model/chain.hpp"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace ros_ethercat_mechanism_model {

using namespace std;

bool Chain::init(Robot *robot_state, const std::string &root, const std::string &tip)
{

  robot_state_ = robot_state;

  // Constructs the kdl chain
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_state->robot_model_, kdl_tree)){
    ROS_ERROR("Could not convert urdf into kdl tree");
    return false;
  }

  bool res;
  try{
    res = kdl_tree.getChain(root, tip, kdl_chain_);
  }
  catch(...){
    res = false;
  }
  if (!res){
    ROS_ERROR("Could not extract chain between %s and %s from kdl tree",
              root.c_str(), tip.c_str());
    return false;
  }


  // Pulls out all the joint indices
  joints_.clear();
  for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None){ 
      JointState* jnt = robot_state->getJointState(kdl_chain_.getSegment(i).getJoint().getName());
      if (!jnt){
        ROS_ERROR("Joint '%s' is not found in joint state vector", kdl_chain_.getSegment(i).getJoint().getName().c_str());
        return false;
      }
      joints_.push_back(jnt);
    }
  }
  ROS_DEBUG("Added %i joints", int(joints_.size()));

  return true;
}

void Chain::getPositions(std::vector<double> &positions)
{
  positions.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    positions[i] = joints_[i]->position_;
  }
}

void Chain::getVelocities(std::vector<double> &velocities)
{
  velocities.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    velocities[i] = joints_[i]->velocity_;
  }
}

void Chain::getEfforts(std::vector<double> &efforts)
{
  efforts.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    efforts[i] = joints_[i]->measured_effort_;
  }
}

bool Chain::allCalibrated()
{
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
      return false;
  }
  return true;
}

void Chain::toKDL(KDL::Chain &chain)
{
  chain = kdl_chain_;
}


void Chain::getPositions(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    a(i) = joints_[i]->position_;
}

void Chain::getVelocities(KDL::JntArrayVel& a)
{
  assert(a.q.rows() == joints_.size());
  assert(a.qdot.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i){
    a.q(i) = joints_[i]->position_;
    a.qdot(i) = joints_[i]->velocity_;
  }
}

void Chain::getEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    a(i) = joints_[i]->measured_effort_;
}

void Chain::setEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ = a(i);
}

void Chain::addEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i]->commanded_effort_ += a(i);
}


JointState *Chain::getJoint(unsigned int actuated_joint_i)
{
  if (actuated_joint_i >= joints_.size())
    return NULL;
  else
    return joints_[actuated_joint_i];
}



}
