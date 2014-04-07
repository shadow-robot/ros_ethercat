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

#include <kdl_parser/kdl_parser.hpp>
#include "ros_ethercat_mechanism_model/tree.hpp"

namespace ros_ethercat_mechanism_model
{

bool Tree::init(Robot *robot_state)
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
  for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end(); ++seg_it)
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

} // namespace

