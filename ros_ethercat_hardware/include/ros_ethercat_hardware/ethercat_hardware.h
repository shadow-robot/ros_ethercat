/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ETHERCAT_HARDWARE_H
#define ETHERCAT_HARDWARE_H

#include <hardware_interface/hardware_interface.h>

#include <ros_ethercat_eml/ethercat_AL.h>
#include <ros_ethercat_eml/ethercat_master.h>
#include <ros_ethercat_eml/ethercat_slave_handler.h>
#include <ros_ethercat_eml/ethercat_dll.h>
#include <ros_ethercat_eml/ethercat_device_addressed_telegram.h>

#include "ros_ethercat_hardware/ethercat_device.h"
#include "ros_ethercat_hardware/ethercat_com.h"
#include "ros_ethercat_hardware/ethernet_interface_info.h"

#include <realtime_tools/realtime_publisher.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <pluginlib/class_loader.h>

#include <std_msgs/Bool.h>

#include <boost/regex.hpp>

using namespace boost::accumulators;

struct EthercatHardwareDiagnostics
{
  EthercatHardwareDiagnostics();
  void resetMaxTiming();
  accumulator_set<double, stats<tag::max, tag::mean> > pack_command_acc_; //!< time taken by all devices packCommand functions
  accumulator_set<double, stats<tag::max, tag::mean> > txandrx_acc_; //!< time taken by to transmit and recieve process data
  accumulator_set<double, stats<tag::max, tag::mean> > unpack_state_acc_; //!< time taken by all devices updateState functions
  accumulator_set<double, stats<tag::max, tag::mean> > publish_acc_; //!< time taken by any publishing step in main loop
  double max_pack_command_;
  double max_txandrx_;
  double max_unpack_state_;
  double max_publish_;
  int txandrx_errors_;
  unsigned device_count_;
  bool pd_error_;
  bool halt_after_reset_; //!< True if motor halt soon after motor reset
  unsigned reset_motors_service_count_; //!< Number of times reset_motor service has been used
  unsigned halt_motors_service_count_; //!< Number of time halt_motor service call is used
  unsigned halt_motors_error_count_; //!< Number of transitions into halt state due to device error
  struct netif_counters counters_;
  bool input_thread_is_stopped_;
  bool motors_halted_; //!< True if motors are halted
  const char* motors_halted_reason_; //!< reason that motors first halted

  static const bool collect_extra_timing_ = true;
};

/*!
 * \brief Publishes EthercatHardware diagnostics.
 *
 * All the string formating used for creating diagnostics is too
 * slow to be run in the realtime thread. Instead, a copy of the raw
 * diagnostics data is made and a separate thread does the string conversion
 * and publishing.
 * Previously, the diagnostics data used by publishing thread was contained
 * in the EthercatHardware class.  However, this allowed the publishing thread
 * access to other non thread-safe data.
 * This class keeps the diagnostics data used by the publish thread explicitly
 * separate.
 */
class EthercatHardwareDiagnosticsPublisher
{
public:

  EthercatHardwareDiagnosticsPublisher(ros::NodeHandle &node);
  ~EthercatHardwareDiagnosticsPublisher();

  /*!
   * \brief Initializes hardware publish.
   * \param buffer_size size of process data buffer
   * \param number of EtherCAT slave devices
   */
  void initialize(const string &interface, unsigned int buffer_size,
                  const std::vector<boost::shared_ptr<EthercatDevice> > &slaves,
                  unsigned int num_ethercat_devices_,
                  unsigned timeout,
                  unsigned max_pd_retries);

  /*!
   * \brief Triggers publishing of new diagnostics data
   *
   * Makes copy of diagnostics data and triggers internal thread to
   * started conversion and publish of data.
   * This function will not block.
   */
  void publish(const unsigned char *buffer, const EthercatHardwareDiagnostics &diagnostics);

  /*!
   * \brief Stops publishing thread.  May block.
   */
  void stop();

private:

  /*!
   * \brief Publishes diagnostics
   *
   * Takes internally saved diagnostics data and converts to a ROS
   * diagnostics status message.
   * This function performs a lot of string formatting, so it is slow.
   */
  void publishDiagnostics();

  /*!
   * \brief Publishing thread main loop
   *
   * Waits for condition variable to start publishing internal data.
   */
  void diagnosticsThreadFunc();

  /*!
   * \brief Helper function for converting timing for diagnostics
   */
  static void timingInformation(diagnostic_updater::DiagnosticStatusWrapper &status,
                                const string &key,
                                const accumulator_set<double, stats<tag::max, tag::mean> > &acc,
                                double max);

  ros::NodeHandle node_;

  boost::mutex diagnostics_mutex_; //!< mutex protects all class data and cond variable
  boost::condition_variable diagnostics_cond_;
  bool diagnostics_ready_;
  boost::thread diagnostics_thread_;

  ros::Publisher publisher_;

  EthercatHardwareDiagnostics diagnostics_; //!< Diagnostics information use by publish function
  unsigned char *diagnostics_buffer_;
  unsigned int buffer_size_;
  std::vector<boost::shared_ptr<EthercatDevice> > slaves_;
  unsigned int num_ethercat_devices_;
  string interface_;

  //! Timeout controls how long EtherCAT driver waits for packet before declaring it as dropped.
  unsigned timeout_;
  //! Number of times (in a row) to retry sending process data (realtime data) before halting motors
  unsigned max_pd_retries_;

  //! Count of dropped packets last diagnostics cycle
  uint64_t last_dropped_packet_count_;
  //! Time last packet was dropped 0 otherwise.  Used for warning about dropped packets.
  ros::Time last_dropped_packet_time_;
  //! Number of seconds since late dropped packet to keep warning
  static const unsigned dropped_packet_warning_hold_time_ = 10; //keep warning up for 10 seconds

  diagnostic_msgs::DiagnosticArray diagnostic_array_;
  //! Information about Ethernet interface used for EtherCAT communication
  EthernetInterfaceInfo ethernet_interface_info_;
  vector<diagnostic_msgs::KeyValue> values_;
  diagnostic_updater::DiagnosticStatusWrapper status_;
};

class EthercatHardware
{
public:
  /*!
   * \brief Scans the network and gives a list of detected devices on a given ethercat port
   * \param eth is the thernet port to be scanned
   */
  static std::vector<EtherCAT_SlaveHandler> scanPort(const std::string& eth);


  /*!
   * \brief Constructor
   */
  EthercatHardware(const std::string& name, hardware_interface::HardwareInterface *hw,
                   const string &eth, bool allow_unprogrammed);

  /*!
   * \brief Destructor
   */
  ~EthercatHardware();

  /*!
   * \brief Send most recent motor commands and retrieve updates. This command must be run at a sufficient rate or else the motors will be disabled.
   * \param reset A boolean indicating if the motor controller boards should be reset
   * \param halt A boolean indicating if the motors should be halted
   */
  void update(bool reset, bool halt);

  /*!
   * \brief Initialize the EtherCAT Master Library.
   */
  void init();

  /*!
   * \brief Collects diagnostics from all devices.
   */
  void collectDiagnostics();

  void printCounters(std::ostream &os = std::cout);

  /*!
   * \brief Send process data
   */
  bool txandrx_PD(unsigned buffer_size, unsigned char* buffer, unsigned tries);

  /*!
   * \brief Ask one or all EtherCAT devices to publish (motor) traces
   * \param position device ring position to publish trace for.  Use -1 to trigger all devices.
   * \param reason Message to put in trace as reason.
   * \param level Level to put in trace (aka ERROR=2, WARN=1, OK=0)
   * \param delay Publish trace after delay cycles.  For 1kHz realtime loop 1cycle = 1ms.
   * \return Return true if device supports publishing trace.  False, if not.
   *         If all devices are triggered, returns true if any device publishes trace.
   */
  bool publishTrace(int position, const string &reason, unsigned level, unsigned delay);

  hardware_interface::HardwareInterface *hw_;

  const std::vector<boost::shared_ptr<const EthercatDevice> > getSlaves() const
  {
    return std::vector<boost::shared_ptr<const EthercatDevice> >(slaves_.begin(), slaves_.end());
  }

private:
  static void changeState(EtherCAT_SlaveHandler *sh, EC_State new_state);

  void loadNonEthercatDevices();
  boost::shared_ptr<EthercatDevice> configNonEthercatDevice(const std::string &product_id,
                                                            const std::string &data);

  void haltMotors(bool error, const char* reason);

  void publishDiagnostics(); //!< Collects raw diagnostics data and passes it to diagnostics_publisher
  static void updateAccMax(double &max, const accumulator_set<double, stats<tag::max, tag::mean> > &acc);
  boost::shared_ptr<EthercatDevice> configSlave(EtherCAT_SlaveHandler *sh);
  bool setRouterToSlaveHandlers();

  ros::NodeHandle node_;

  struct netif *ni_;

  string interface_; //!< The socket interface that is connected to the EtherCAT devices (e.g., eth0)

  EtherCAT_DataLinkLayer m_dll_instance_;
  EC_Logic m_logic_instance_;
  EtherCAT_PD_Buffer pd_buffer_;
  EtherCAT_AL *application_layer_;
  EtherCAT_Router *m_router_;
  EtherCAT_Master *ethercat_master_;

  std::vector<boost::shared_ptr<EthercatDevice> > slaves_;
  unsigned int num_ethercat_devices_;

  unsigned char *this_buffer_;
  unsigned char *prev_buffer_;
  unsigned char *buffers_;
  unsigned int buffer_size_;

  bool halt_motors_;
  unsigned int reset_state_;

  unsigned timeout_; //!< Timeout (in microseconds) to used for sending/recieving packets once in realtime mode.
  unsigned max_pd_retries_; //!< Max number of times to retry sending process data before halting motors

  EthercatHardwareDiagnostics diagnostics_;
  EthercatHardwareDiagnosticsPublisher diagnostics_publisher_;
  ros::Time last_published_;
  ros::Time last_reset_;

  realtime_tools::RealtimePublisher<std_msgs::Bool> motor_publisher_;

  EthercatOobCom *oob_com_;

  pluginlib::ClassLoader<EthercatDevice> device_loader_;

  bool allow_unprogrammed_; //!< if the driver should treat the discovery of unprogrammed boards as a fatal error. Set to 'true' during board configuration, and set to 'false' otherwise.

  int start_address_;
};

#endif /* ETHERCAT_HARDWARE_H */
