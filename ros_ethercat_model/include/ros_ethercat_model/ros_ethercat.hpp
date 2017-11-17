/*
 * ros_ethercat.hpp
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

#ifndef ROS_ETHERCAT_MODEL_ROS_ETHERCAT_HPP_
#define ROS_ETHERCAT_MODEL_ROS_ETHERCAT_HPP_

#include <fcntl.h>
#include <sys/stat.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <sr_robot_msgs/MechanismStatistics.h>
#include "ros_ethercat_model/robot_state.hpp"
#include "ros_ethercat_model/robot_state_interface.hpp"
#include "ros_ethercat_model/mech_stats_publisher.hpp"
#include "ros_ethercat_hardware/ethercat_hardware.h"
#include "ros_ethercat_model/imu_state.hpp"


/** \brief Contains robot state information and init, read, write function.
 *
 * The robot state is contained in ros_ethercat_model::RobotStateState object
 * as used by pr2_controller object. eNvertheless, the main loop in main.cpp
 * instantiates a ros_control controller_manager. So a pr2_controller with few modifications
 * may be loaded with controller_manager with RobotState as a custom interface.
 *
 * ros_control interfaces are exposed alongside RobotState. So controllers from
 * ros_controllers package may also be loaded. These new interfaces contain pointers
 * to data in RobotState so there is no copying or data redundancy.
 *
 * The read and write functions will call the propagate functions of pr2_transmissions.
 * Hardware read and write takes place in the EthercatHardware object in main.cpp
 *
 * initXml, read and write should be called inside main.cpp
 */

using std::string;
using std::vector;
using boost::ptr_unordered_map;
using boost::ptr_vector;
using ros_ethercat_model::JointState;
using ros_ethercat_model::Actuator;
using ros_ethercat_model::Transmission;
using ros_ethercat_model::CustomHW;

static const string name = "ros_ethercat";

class RosEthercat : public hardware_interface::RobotHW
{
public:
  RosEthercat() :
    compatibility_mode_(false),
    collect_diagnostics_running_(false),
    run_diagnostics_(false)
  {

  }

  RosEthercat(ros::NodeHandle &nh, const string &eth, bool allow, TiXmlElement* config) :
    compatibility_mode_(true),
    collect_diagnostics_running_(false),
    run_diagnostics_(false)
  {
    model_.reset(new RobotState(config));
    vector<string> port_names;
    boost::split(port_names, eth, boost::is_any_of("_, "));
    for (vector<string>::const_iterator port_name = port_names.begin();
         port_name != port_names.end();
         ++port_name)
    {
      if (!port_name->empty())
      {
        ethercat_hardware_.push_back(new EthercatHardware(name,
                                                          static_cast<hardware_interface::HardwareInterface*> (model_.get()),
                                                          *port_name,
                                                          allow));
        ROS_INFO_STREAM("Added Ethernet port " << *port_name);
      }
    }

    for (ptr_unordered_map<string, ros_ethercat_model::ImuState>::iterator it = model_->imu_states_.begin();
         it != model_->imu_states_.end(); ++it)
    {
      ROS_INFO_STREAM("IMU State Interface for IMU " << it->first);
      hardware_interface::ImuSensorHandle imu_sh(it->second->data_);
      imu_sensor_interface_.registerHandle(imu_sh);
    }

    for (ptr_unordered_map<string, JointState>::iterator it = model_->joint_states_.begin();
         it != model_->joint_states_.end();
         ++it)
    {
      hardware_interface::JointStateHandle jsh(it->first,
                                               &it->second->position_,
                                               &it->second->velocity_,
                                               &it->second->effort_);
      joint_state_interface_.registerHandle(jsh);

      joint_position_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
                                                                                       & it->second->commanded_position_));
      joint_velocity_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
                                                                                       & it->second->commanded_velocity_));
      joint_effort_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
                                                                                     & it->second->commanded_effort_));
    }

    if (!model_->joint_states_.empty())
      mech_stats_publisher_.reset(new MechStatsPublisher(nh, *model_));

    robot_state_interface_.registerHandle(ros_ethercat_model::RobotStateHandle("unique_robot_hw", model_.get()));

    registerInterface(&robot_state_interface_);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_command_interface_);
    registerInterface(&joint_velocity_command_interface_);
    registerInterface(&joint_effort_command_interface_);
    registerInterface(&imu_sensor_interface_);

  }

  virtual ~RosEthercat()
  {
    if (!compatibility_mode_)
    {
      // Shutdown all of the motors on exit
      shutdown();
      // Cleanup pid files
      cleanupPidFile(NULL);
      cleanupPidFile(eth_.c_str());

      stop_collect_diagnostics();
      while (is_collect_diagnostics_running())
      {
        usleep(100);
      }
    }
  }

  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    // Load robot description
    TiXmlDocument xml;
    TiXmlElement *root;
    TiXmlElement *root_element;

    std::string robot_description;
    std::string robot_description_param;
    bool allow;

    robot_state_name_ = robot_hw_nh.getNamespace().substr(1); // remove the leading slash of the namespace
    ROS_INFO_STREAM("Robot State Name: " << robot_state_name_);

    if (!robot_hw_nh.getParam("robot_description_param", robot_description_param))
    {
      ROS_ERROR("robot_description_param not found (namespace: %s)", robot_hw_nh.getNamespace().c_str());
      return false;
    }

    if (robot_description_param == "None")
    {
      root = NULL;
    }
    else
    {
      if (!root_nh.getParam(robot_description_param, robot_description))
      {
        ROS_ERROR("Robot description: %s not found (namespace: %s)", robot_description_param.c_str(), root_nh.getNamespace().c_str());
        return false;
      }
      xml.Parse(robot_description.c_str());
      root_element = xml.RootElement();
      root = xml.FirstChildElement("robot");
      if (!root || !root_element)
      {
        ROS_ERROR("Robot description %s has no root",robot_description_param.c_str());
        return false;
      }
    }

    vector<string> joint_filter;
    !robot_hw_nh.getParam("joint_filter", joint_filter);

    model_.reset(new RobotState(root, joint_filter));

    if (!robot_hw_nh.getParam("ethercat_port", eth_))
    {
      ROS_ERROR("ethercat_port param not found (namespace: %s)", robot_hw_nh.getNamespace().c_str());
      return false;
    }

    // EtherCAT lock for this interface (e.g. Ethernet port)
    if (setupPidFile(eth_.c_str()) < 0)
    {
      return false;
    }

    robot_hw_nh.param<bool>("allow_unprogrammed", allow, false);

    vector<string> port_names;
    boost::split(port_names, eth_, boost::is_any_of("_, "));
    for (vector<string>::const_iterator port_name = port_names.begin();
         port_name != port_names.end();
         ++port_name)
    {
      if (!port_name->empty())
      {
        ethercat_hardware_.push_back(new EthercatHardware(name,
                                                          static_cast<hardware_interface::HardwareInterface*> (model_.get()),
                                                          *port_name,
                                                          allow));
        ROS_INFO_STREAM("Added Ethernet port " << *port_name);
      }
    }

    for (ptr_unordered_map<string, ros_ethercat_model::ImuState>::iterator it = model_->imu_states_.begin();
         it != model_->imu_states_.end(); ++it)
    {
      ROS_INFO_STREAM("IMU State Interface for IMU " << it->first);
      hardware_interface::ImuSensorHandle imu_sh(it->second->data_);
      imu_sensor_interface_.registerHandle(imu_sh);
    }

    for (ptr_unordered_map<string, JointState>::iterator it = model_->joint_states_.begin();
         it != model_->joint_states_.end();
         ++it)
    {
      // joints have already had filter applied in initialisation of robot model
      ROS_INFO_STREAM("Joint state interface for hand joint " << it->first);
      hardware_interface::JointStateHandle jsh(it->first,
					       &it->second->position_,
					       &it->second->velocity_,
					       &it->second->effort_);
      joint_state_interface_.registerHandle(jsh);

      joint_position_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
										       &it->second->commanded_position_));
      joint_velocity_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
										       &it->second->commanded_velocity_));
      joint_effort_command_interface_.registerHandle(hardware_interface::JointHandle(jsh,
										     &it->second->commanded_effort_));
    }

    if (!model_->joint_states_.empty())
      mech_stats_publisher_.reset(new MechStatsPublisher(root_nh, *model_));

    robot_state_interface_.registerHandle(ros_ethercat_model::RobotStateHandle(robot_state_name_, model_.get()));

    registerInterface(&robot_state_interface_);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_command_interface_);
    registerInterface(&joint_velocity_command_interface_);
    registerInterface(&joint_effort_command_interface_);
    registerInterface(&imu_sensor_interface_);

    // Start a thread to collect diagnostics. This could actually be inside the EthercatHardware class
    // but until we remove the compatibility mode this will do.
    collect_diagnostics_thread_ = boost::thread(&RosEthercat::collect_diagnostics_loop, this);

    return true;
  }

  /// propagate position actuator -> joint and set commands to zero
  void read(const ros::Time &time, const ros::Duration& period)
  {
    for (ptr_vector<EthercatHardware>::iterator eh = ethercat_hardware_.begin();
         eh != ethercat_hardware_.end();
         ++eh)
    {
      eh->update(false, false);
    }

    model_->current_time_ = time;
    model_->propagateActuatorPositionToJointPosition();

    for (ptr_unordered_map<std::string, CustomHW>::iterator it = model_->custom_hws_.begin();
         it != model_->custom_hws_.end();
         ++it)
    {
      it->second->read(time);
    }

    for (ptr_unordered_map<string, JointState>::iterator it = model_->joint_states_.begin();
         it != model_->joint_states_.end();
         ++it)
    {
      it->second->joint_statistics_.update(it->second);
      it->second->commanded_effort_ = 0;
    }
  }

  /// propagate effort joint -> actuator and enforce safety limits
  void write(const ros::Time &time, const ros::Duration& period)
  {
    /// Modify the commanded_effort_ of each joint state so that the joint limits are satisfied
    for (ptr_unordered_map<string, JointState>::iterator it = model_->joint_states_.begin();
         it != model_->joint_states_.end();
         ++it)
    {
      it->second->enforceLimits();
    }

    model_->propagateJointEffortToActuatorEffort();

    for (ptr_unordered_map<std::string, CustomHW>::iterator it = model_->custom_hws_.begin();
         it != model_->custom_hws_.end();
         ++it)
    {
      it->second->write(time);
    }

    if (!model_->joint_states_.empty())
      mech_stats_publisher_->publish(time);
  }

  /// stop all actuators
  void shutdown()
  {
    for (ptr_vector<Transmission>::iterator it = model_->transmissions_.begin();
         it != model_->transmissions_.end();
         ++it)
    {
      it->actuator_->command_.enable_ = false;
      it->actuator_->command_.effort_ = 0;
    }

    for (ptr_vector<EthercatHardware>::iterator eh = ethercat_hardware_.begin();
         eh != ethercat_hardware_.end();
         ++eh)
    {
      eh->update(false, true);
    }
  }

  static const string pid_dir;
  string eth_;
  boost::shared_ptr<ros_ethercat_model::RobotState> model_;
  ptr_vector<EthercatHardware> ethercat_hardware_;
  boost::scoped_ptr<MechStatsPublisher> mech_stats_publisher_;

  // robot state interface
  ros_ethercat_model::RobotStateInterface robot_state_interface_;

  // joint state interface
  hardware_interface::JointStateInterface joint_state_interface_;

  // imu sensor interface
  hardware_interface::ImuSensorInterface imu_sensor_interface_;

  // joint command interface
  hardware_interface::PositionJointInterface joint_position_command_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_command_interface_;
  hardware_interface::EffortJointInterface joint_effort_command_interface_;

protected:
  static int lock_fd(int fd)
  {
    struct flock lock;

    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;

    return fcntl(fd, F_SETLK, &lock);
  }

  static string generatePIDFilename(const char* interface)
  {
    string filename;
    filename = pid_dir + "EtherCAT_" + string(interface) + ".pid";
    return filename;
  }

  static int setupPidFile(const char* interface)
  {
    pid_t pid;
    int fd;
    FILE *fp = NULL;

    string filename = generatePIDFilename(interface);

    umask(0);
    mkdir(pid_dir.c_str(), 0777);
    int PID_FLAGS = O_RDWR | O_CREAT | O_EXCL;
    int PID_MODE = S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH;
    fd = open(filename.c_str(), PID_FLAGS, PID_MODE);
    if (fd == -1)
    {
      if (errno != EEXIST)
      {
        ROS_FATAL("Unable to create pid file '%s': %s", filename.c_str(), strerror(errno));
        return -1;
      }

      if ((fd = open(filename.c_str(), O_RDWR)) < 0)
      {
        ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
        return -1;
      }

      if ((fp = fdopen(fd, "rw")) == NULL)
      {
        ROS_FATAL("Can't read from '%s': %s", filename.c_str(), strerror(errno));
        return -1;
      }
      pid = -1;
      if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
      {
        int rc;

        if ((rc = unlink(filename.c_str())) == -1)
        {
          ROS_FATAL("Can't remove stale pid file '%s': %s", filename.c_str(), strerror(errno));
          return -1;
        }
      }
      else
      {
        ROS_FATAL("Another instance of ros_ethercat is already running with pid: %d", pid);
        return -1;
      }
    }

    unlink(filename.c_str());
    fd = open(filename.c_str(), PID_FLAGS, PID_MODE);

    if (fd == -1)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
      return -1;
    }

    if (lock_fd(fd) == -1)
    {
      ROS_FATAL("Unable to lock pid file '%s': %s", filename.c_str(), strerror(errno));
      return -1;
    }

    if ((fp = fdopen(fd, "w")) == NULL)
    {
      ROS_FATAL("fdopen failed: %s", strerror(errno));
      return -1;
    }

    fprintf(fp, "%d\n", getpid());

    /* We do NOT close fd, since we want to keep the lock. */
    fflush(fp);
    fcntl(fd, F_SETFD, (long) 1);

    return 0;
  }

  static void cleanupPidFile(const char* interface)
  {
    string filename = generatePIDFilename(interface);
    unlink(filename.c_str());
  }

  void collect_diagnostics_loop()
  {
    run_diagnostics_ = true;
    collect_diagnostics_running_ = true;
    ros::Rate diag_rate(1.0); // Send diagnostics at 1Hz
    while (run_diagnostics_)
    {
      for (ptr_vector<EthercatHardware>::iterator eh = ethercat_hardware_.begin(); eh != ethercat_hardware_.end(); ++eh)
      {
        eh->collectDiagnostics();
      }
      diag_rate.sleep();
    }
    collect_diagnostics_running_ = false;
  }

protected:

  void stop_collect_diagnostics()
  {
    run_diagnostics_ = false;
  }

  bool is_collect_diagnostics_running()
  {
    return collect_diagnostics_running_;
  }

  bool compatibility_mode_;
  bool run_diagnostics_;
  bool collect_diagnostics_running_;
  boost::thread collect_diagnostics_thread_;
  std::string robot_state_name_;
};

const string RosEthercat::pid_dir = "/var/tmp/run/";

#endif
