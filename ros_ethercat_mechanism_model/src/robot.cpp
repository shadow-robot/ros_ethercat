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
using std::cout;

using pluginlib::ClassLoader;

Robot::Robot(TiXmlElement *root) :
    transmission_loader_("ros_ethercat_mechanism_model", "ros_ethercat_mechanism_model::Transmission")
{
  try
  {
    // Parses the xml into a robot model
    if (!robot_model_.initXml(root))
      throw runtime_error("Failed to load robot_model_");
    cout << "Loaded robot model\n";

    // Constructs the transmissions by parsing custom xml.
    TiXmlElement *xit = NULL;
    for (xit = root->FirstChildElement("transmission");
         xit;
         xit = xit->NextSiblingElement("transmission"))
    {
      string type(xit->Attribute("type"));

      Transmission *t = transmission_loader_.createUnmanagedInstance(type);
      if (!t)
        throw runtime_error(string("Unknown transmission type: ") + type);
      cout << "Loading transmission type " << type << '\n';
      if (t->initXml(xit, this))
      {
        transmissions_.push_back(t);

        // Add actuator names to transmission
        vector<Actuator*> acts;

        for (vector<string>::iterator it = t->actuator_names_.begin(); it != t->actuator_names_.end(); ++it)
        {
          Actuator *act = getActuator(*it);
          if (act)
          {
            acts.push_back(act);
            cout << "Found actuator " << *it << " for transmission " << t->name_ << '\n';
          }
          else
            ROS_ERROR_STREAM("Transmission " << t->name_ << " contains actuator " << *it << " that is undefined");
        }
        transmissions_in_.push_back(acts);

        // Wires up the transmissions to the joint state
        vector<JointState*> stats;
        for (vector<string>::iterator it = t->joint_names_.begin(); it != t->joint_names_.end(); ++it)
        {
          joint_states_[*it].joint_ = robot_model_.getJoint(*it);
          stats.push_back(&joint_states_[*it]);
          cout << "Wired joint " << *it << " to transmission " << t->name_ << '\n';
        }
        transmissions_out_.push_back(stats);
      }
      else
        ROS_FATAL_STREAM("Failed to initialize transmission type: " + type);
    }

    // warnings
    if (transmissions_.empty())
      ROS_WARN("No transmissions were specified in the robot description.");
  }
  catch (const runtime_error &ex)
  {
    ROS_FATAL_STREAM("Mechanism Model failed to parse the URDF xml into a robot model\n" << ex.what());
  }
  catch (...)
  {
    ROS_FATAL("unknown error");
  }
  cout << "Number of transmissions found in robot = " << transmissions_.size() << '\n';
  cout << "Number of actuators found in robot = " << actuators_.size() << '\n';
  cout << "Number of joint states found in robot = " << joint_states_.size() << '\n';
}

JointState* Robot::getJointState(const string &name)
{
  if (joint_states_.count(name))
    return &joint_states_[name];
  else
    return NULL;
}

Actuator* Robot::getActuator(const std::string &name)
{
  if (actuators_.count(name))
    return &actuators_[name];
  else
    return NULL;
}

CustomHW* Robot::getCustomHW(const std::string &name)
{
  if (custom_hws_.count(name))
    return &custom_hws_[name];
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

void Robot::propagateActuatorPositionToJointPosition()
{
  for (size_t i = 0; i < transmissions_.size(); ++i)
    transmissions_[i].propagatePosition(transmissions_in_[i], transmissions_out_[i]);
}

void Robot::propagateJointEffortToActuatorEffort()
{
  for (size_t i = 0; i < transmissions_.size(); ++i)
    transmissions_[i].propagateEffort(transmissions_out_[i], transmissions_in_[i]);
}
