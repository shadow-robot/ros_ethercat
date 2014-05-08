/*
 * robot_state.hpp
 *
 *  Created on: 7 Jan 2014
 *      Author: Manos Nikolaidis
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Shadow Robot Company Ltd.
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

#ifndef ROS_ETHERCAT_MODEL_ROBOTSTATE_HPP
#define ROS_ETHERCAT_MODEL_ROBOTSTATE_HPP

#include <urdf/model.h>
#include <pluginlib/class_loader.h>
#include <boost/ptr_container/ptr_unordered_map.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <hardware_interface/hardware_interface.h>
#include "ros_ethercat_model/joint.hpp"
#include "ros_ethercat_model/transmission.hpp"
#include "ros_ethercat_model/hardware_interface.hpp"

namespace ros_ethercat_model
{
/** \brief This class provides the controllers with an interface to the robot state
 *
 * Most controllers that need the robot state should use the joint states, to get
 * access to the joint position/velocity/effort, and to command the effort a joint
 * should apply. Controllers can get access to the hard realtime clock through current_time_
 *
 * A CustomHW class to add arbitrary hardware devices
 */
class RobotState : public hardware_interface::HardwareInterface
{
public:
  RobotState(TiXmlElement *root)
    : transmission_loader_("ros_ethercat_model", "ros_ethercat_model::Transmission")
  {
    if (root)
      initXml(root);
  }

  void initXml(TiXmlElement *root)
  {
    try
    {
      if (!robot_model_.initXml(root))
        throw std::runtime_error("Failed to load robot_model_");

      for (TiXmlElement *xit = root->FirstChildElement("transmission"); xit; xit = xit->NextSiblingElement("transmission"))
      {
        std::string type(xit->Attribute("type"));

        Transmission *t = transmission_loader_.createUnmanagedInstance(type);
        if (!t || !t->initXml(xit, this))
          throw std::runtime_error(std::string("Failed to initialize transmission type: ") + type);
        transmissions_.push_back(t);

        std::vector<Actuator*> acts;
        for (std::vector<std::string>::iterator it = t->actuator_names_.begin(); it != t->actuator_names_.end(); ++it)
        {
          Actuator *act = getActuator(*it);
          if (!act)
            throw std::runtime_error(std::string("Transmission ") + t->name_ + " contains undefined actuator " + *it);
          acts.push_back(act);
        }
        transmissions_in_.push_back(acts);

        std::vector<JointState*> stats;
        for (std::vector<std::string>::iterator it = t->joint_names_.begin(); it != t->joint_names_.end(); ++it)
        {
          if (!robot_model_.getJoint(*it))
            throw std::runtime_error(std::string("Failed to find in robot model transmission's joint named: ") + type);
          joint_states_[*it].joint_ = robot_model_.getJoint(*it);
          stats.push_back(&joint_states_[*it]);
        }
        transmissions_out_.push_back(stats);
      }
    }
    catch (const std::runtime_error &ex)
    {
      ROS_FATAL_STREAM("ros_ethercat_model failed to parse the URDF xml into a robot model\n" << ex.what());
    }
  }

  /// Propagete the actuator positions, through the transmissions, to the joint positions
  void propagateActuatorPositionToJointPosition()
  {
    for (size_t i = 0; i < transmissions_.size(); ++i)
      transmissions_[i].propagatePosition(transmissions_in_[i], transmissions_out_[i]);
  }

  /// Propagete the joint efforts, through the transmissions, to the actuator efforts
  void propagateJointEffortToActuatorEffort()
  {
    for (size_t i = 0; i < transmissions_.size(); ++i)
      transmissions_[i].propagateEffort(transmissions_out_[i], transmissions_in_[i]);
  }

  /// get an actuator by actuator name or NULL on failure
  Actuator* getActuator(const std::string &name)
  {
    return actuators_.count(name) ? &actuators_[name] : NULL;
  }

  /// Get Custom Hardware device by name or NULL on failure
  CustomHW* getCustomHW(const std::string &name)
  {
    return custom_hws_.count(name) ? &custom_hws_[name] : NULL;
  }

  /// Get a joint state by name or NULL on failure
  JointState* getJointState(const std::string &name)
  {
    return joint_states_.count(name) ? &joint_states_[name] : NULL;
  }

  ros::Time getTime()
  {
    return current_time_;
  }

  /// The time at which the commands were sent to the hardware
  ros::Time current_time_;

  // for each transmission cache pointers to actuators and joints that it connects
  std::vector<std::vector<Actuator*> > transmissions_in_;
  std::vector<std::vector<JointState*> > transmissions_out_;

  /// The joint states mapped to the joint names
  boost::unordered_map<std::string, JointState> joint_states_;

  /// The actuators mapped to their names
  boost::ptr_unordered_map<std::string, Actuator> actuators_;

  /// Custom hardware structures mapped to their names
  boost::ptr_unordered_map<std::string, CustomHW> custom_hws_;

  /// The kinematic/dynamic model of the robot
  urdf::Model robot_model_;

  boost::ptr_vector<Transmission> transmissions_;
  pluginlib::ClassLoader<Transmission> transmission_loader_;
};
}
#endif
