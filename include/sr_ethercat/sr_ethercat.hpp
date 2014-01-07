/*
 * sr_ethercat_interface.hpp
 *
 *  Created on: 7 Jan 2014
 *      Author: manos
 */

#ifndef SR_ETHERCAT_INTERFACE_HPP_
#define SR_ETHERCAT_INTERFACE_HPP_

#include <string>
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <pr2_mechanism_model/robot.h>
#include "sr_ethercat/controller_spec.h"
#include <pr2_mechanism_msgs/ListControllerTypes.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/ReloadControllerLibraries.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <controller_manager/controller_manager.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <std_srvs/Empty.h>

class sr_ethercat : public hardware_interface::RobotHW
{
public:
  sr_ethercat(pr2_hardware_interface::HardwareInterface *hw, ros::NodeHandle nh) :
    model_(hw),
    state_(NULL),
    controller_node_(nh),
    cm_node_(nh, "controller_manager"),
    pub_joint_state_(nh, "joint_states", 1),
    pub_mech_stats_(nh, "mechanism_statistics", 1),
    last_published_joint_state_(ros::Time::now()),
    last_published_mechanism_stats_(ros::Time::now())
  {}

  virtual ~sr_ethercat()
  {
    if (state_)
      delete state_;
  }

  bool configure();
  bool initXml(TiXmlElement* config);

  pr2_mechanism_model::Robot model_;
  pr2_mechanism_model::RobotState *state_;

private:
  ros::NodeHandle controller_node_, cm_node_;

  // for controller statistics
  Statistics pre_update_stats_;
  Statistics update_stats_;
  Statistics post_update_stats_;

  // for publishing constroller state
  void publishJointState();
  void publishMechanismStatistics();
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state_;
  realtime_tools::RealtimePublisher<pr2_mechanism_msgs::MechanismStatistics> pub_mech_stats_;
  ros::Duration publish_period_joint_state_, publish_period_mechanism_stats_;
  ros::Time last_published_joint_state_, last_published_mechanism_stats_;

  int current_controllers_list_, used_by_realtime_;
  std::vector<ControllerSpec> controllers_lists_[2];

  bool motors_previously_halted_;
};

#endif /* SR_ETHERCAT_INTERFACE_HPP_ */
