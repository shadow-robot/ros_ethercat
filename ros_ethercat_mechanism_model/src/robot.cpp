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

/*
 * Author: Stuart Glaser, Wim Meeussen
 */
#include <numeric>
#include "ros_ethercat_mechanism_model/robot.hpp"
#include <tinyxml.h>
#include <urdf/model.h>


using namespace ros_ethercat_mechanism_model;
using std::vector;
using std::string;
using std::runtime_error;
using boost::unordered_map;
using boost::shared_ptr;

using pluginlib::ClassLoader;

Robot::Robot(TiXmlElement *root) :
    transmission_loader_("ros_ethercat_mechanism_model", "ros_ethercat_mechanism_model::Transmission")
{
  // Parses the xml into a robot model
  if (!robot_model_.initXml(root)){
    ROS_ERROR("Mechanism Model failed to parse the URDF xml into a robot model");
    return;
  }

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement *xit = NULL;
  size_t actuators_number = 0;
  for (xit = root->FirstChildElement("transmission");
       xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    string type(xit->Attribute("type"));

    try
    {
      Transmission *t = transmission_loader_.createUnmanagedInstance(type);
      if (!t)
        ROS_ERROR("Unknown transmission type: %s", type.c_str());
      else if (!t->initXml(xit, this)){
        ROS_ERROR("Failed to initialize transmission");
      }
      else // Success!
      {
        transmissions_.push_back(*t);
        delete t;

        // Creates a joint state for each transmission and
        vector<Actuator*> acts;

        for (vector<string>::iterator it = t->actuator_names_.begin(); it != t->actuator_names_.end(); ++it)
        {
          acts.push_back(getActuator(*it));
          ++actuators_number;
        }
        transmissions_in_.push_back(acts);

        // Wires up the transmissions to the joint state
        vector<JointState*> stats;
        for (vector<string>::iterator it = t->joint_names_.begin(); it != t->joint_names_.end(); ++it)
        {
          joint_states_[*it].joint_ = robot_model_.getJoint(*it);
          stats.push_back(&joint_states_[*it]);
        }
        transmissions_out_.push_back(stats);
      }
    }
    catch (const runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }
  }

  // warnings
  if (transmissions_.empty())
    ROS_WARN("No transmissions were specified in the robot description.");
  if (actuators_number == 0)
    ROS_WARN("None of the joints in the robot desription matches up to a motor. The robot is uncontrollable.");
}

Transmission *Robot::getTransmission(const string &name) const
{
  for (size_t j = 0; j < transmissions_.size(); ++j)
  {
    if (transmissions_[j].name_ == name)
      return transmissions_[j];
  }

  return NULL;
}

JointState *Robot::getJointState(const string &name) const
{
  if (joint_states_.count(name))
    return &joint_states_[name];
  else
    return NULL;
}

Actuator* Robot::getActuator(const std::string &name) const
{
  if (actuators_.count(name))
    return actuators_[name];
  else
    return NULL;
}

CustomHW* Robot::getCustomHW(const std::string &name) const
{
  if (custom_hws_.count(name))
    return custom_hws_[name];
  else
    return NULL;
}

void Robot::propagateJointPositionToActuatorPosition()
{
  for (size_t i = 0; i < transmissions_.size(); ++i)
    transmissions_[i].propagatePositionBackwards(transmissions_out_[i], transmissions_in_[i]);
}

void Robot::propagateActuatorEffortToJointEffort()
{
  for (size_t i = 0; i < transmissions_.size(); ++i)
    transmissions_[i].propagateEffortBackwards(transmissions_in_[i], transmissions_out_[i]);
}


