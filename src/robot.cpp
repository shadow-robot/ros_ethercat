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

#include "ros_ethercat/robot.h"
#include "ros_ethercat/transmission.h"
#include <tinyxml.h>
#include <urdf/model.h>
#include <pluginlib/class_loader.h>
#include "ros_ethercat/hardware_interface.hpp"


using namespace ros_ethercat_mechanism_model;
using namespace ros_ethercat_hardware_interface;


Robot::Robot(HardwareInterface *hw)
  :hw_(hw)
{}


bool Robot::initXml(TiXmlElement *root)
{
  // check if current time is valid
  if (!hw_){
    ROS_ERROR("Mechanism Model received an invalid hardware interface");
    return false;
  }

  // Parses the xml into a robot model
  if (!robot_model_.initXml(root)){
    ROS_ERROR("Mechanism Model failed to parse the URDF xml into a robot model");
    return false;
  }

  // Creates the plugin loader for transmissions.
  transmission_loader_.reset(new pluginlib::ClassLoader<ros_ethercat_mechanism_model::Transmission>(
                               "ros_ethercat_mechanism_model", "ros_ethercat_mechanism_model::Transmission"));

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement *xit = NULL;
  for (xit = root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    std::string type(xit->Attribute("type"));
    boost::shared_ptr<Transmission> t;
    try {
      // Backwards compatibility for using non-namespaced controller types
      if (!transmission_loader_->isClassAvailable(type))
      {
        std::vector<std::string> classes = transmission_loader_->getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
          if(type == transmission_loader_->getName(classes[i]))
          {
            ROS_WARN("The deprecated transmission type %s was not found.  Using the namespaced version %s instead.  "
                     "Please update your urdf file to use the namespaced version.",
                     type.c_str(), classes[i].c_str());
            type = classes[i];
            break;
          }
        }
      }
      t = transmission_loader_->createInstance(type);
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }

    if (!t)
      ROS_ERROR("Unknown transmission type: %s", type.c_str());
    else if (!t->initXml(xit, this)){
      ROS_ERROR("Failed to initialize transmission");
    }
    else // Success!
      transmissions_.push_back(t);
  }

  return true;
}

ros::Time Robot::getTime()
{
  return hw_->current_time_;
}

template <class T>
int findIndexByName(const std::vector<boost::shared_ptr<T> >& v, 
      const std::string &name)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
      return i;
  }
  return -1;
}

int Robot::getTransmissionIndex(const std::string &name) const
{
  return findIndexByName(transmissions_, name);
}

Actuator* Robot::getActuator(const std::string &name) const
{
  return hw_->getActuator(name);
}

boost::shared_ptr<Transmission> Robot::getTransmission(const std::string &name) const
{
  int i = getTransmissionIndex(name);
  return i >= 0 ? transmissions_[i] : boost::shared_ptr<Transmission>();
}





RobotState::RobotState(Robot *model)
  : model_(model)
{
  assert(model_);

  transmissions_in_.resize(model->transmissions_.size());
  transmissions_out_.resize(model->transmissions_.size());

  // Creates a joint state for each transmission
  unsigned int js_size = 0;
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
     boost::shared_ptr<Transmission> t = model_->transmissions_[i];
    for (unsigned int j = 0; j < t->actuator_names_.size(); ++j)
    {
      Actuator *act = model_->getActuator(t->actuator_names_[j]);
      assert(act != NULL);
      transmissions_in_[i].push_back(act);
    }
    js_size += t->joint_names_.size();
  }

  // Wires up the transmissions to the joint state
  joint_states_.resize(js_size);
  unsigned int js_id = 0;
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
     boost::shared_ptr<Transmission> t = model_->transmissions_[i];
    for (unsigned int j = 0; j < t->joint_names_.size(); ++j)
    {
      joint_states_[js_id].joint_ = model_->robot_model_.getJoint(t->joint_names_[j]);
      joint_states_map_[t->joint_names_[j]] = &(joint_states_[js_id]);
      transmissions_out_[i].push_back(&(joint_states_[js_id]));
      js_id++;
    }
  }

  // warnings
  if (model_->transmissions_.empty())
    ROS_WARN("No transmissions were specified in the robot description.");
  if (js_size == 0)
    ROS_WARN("None of the joints in the robot desription matches up to a motor. The robot is uncontrollable.");
}


JointState *RobotState::getJointState(const std::string &name)
{
  std::map<std::string, JointState*>::iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

const JointState *RobotState::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

void RobotState::propagateActuatorPositionToJointPosition()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePosition(transmissions_in_[i],
                                                 transmissions_out_[i]);
  }

  for (unsigned int i = 0; i < joint_states_.size(); i++)
  {
    joint_states_[i].joint_statistics_.update(&(joint_states_[i]));
  }
}

void RobotState::propagateJointEffortToActuatorEffort()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffort(transmissions_out_[i],
                                               transmissions_in_[i]);
  }
}

bool RobotState::isHalted()
{
  for (unsigned int t = 0; t < transmissions_in_.size(); ++t){
    for (unsigned int a = 0; a < transmissions_in_[t].size(); a++){
      if (transmissions_in_[t][a]->state_.halted_)
        return true;
    }
  }

  return false;
}

void RobotState::enforceSafety()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
  {
    joint_states_[i].enforceLimits();
  }
}

void RobotState::zeroCommands()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
    joint_states_[i].commanded_effort_ = 0;
}

void RobotState::propagateJointPositionToActuatorPosition()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePositionBackwards(transmissions_out_[i],
                                                          transmissions_in_[i]);
  }
}

void RobotState::propagateActuatorEffortToJointEffort()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffortBackwards(transmissions_in_[i],
                                                        transmissions_out_[i]);
  }
}


