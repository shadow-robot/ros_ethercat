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

#include "ros_ethercat_hardware/ethercat_hardware.h"

#include <ros_ethercat_eml/ethercat_xenomai_drv.h>

#include <sstream>

#include <net/if.h>
#include <sys/ioctl.h>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

EthercatHardwareDiagnostics::EthercatHardwareDiagnostics() :
  txandrx_errors_(0),
  device_count_(0),
  pd_error_(false),
  halt_after_reset_(false),
  reset_motors_service_count_(0),
  halt_motors_service_count_(0),
  halt_motors_error_count_(0),
  motors_halted_(false),
  motors_halted_reason_("")
{
  resetMaxTiming();
}

void EthercatHardwareDiagnostics::resetMaxTiming()
{
  max_pack_command_ = 0.0;
  max_txandrx_ = 0.0;
  max_unpack_state_ = 0.0;
  max_publish_ = 0.0;
}

EthercatHardware::EthercatHardware(const std::string& name,
                                   hardware_interface::HardwareInterface *hw,
                                   const std::string& eth,
                                   bool allow_unprogrammed) :
  hw_(hw),
  node_(ros::NodeHandle(name)),
  ni_(NULL),
  interface_(eth),
  pd_buffer_(&m_logic_instance_, &m_dll_instance_),
  this_buffer_(NULL),
  prev_buffer_(NULL),
  buffer_size_(0),
  halt_motors_(true),
  reset_state_(0),
  max_pd_retries_(10),
  diagnostics_publisher_(node_),
  motor_publisher_(node_, "motors_halted", 1, true),
  device_loader_("ros_ethercat_hardware", "EthercatDevice"),
  allow_unprogrammed_(allow_unprogrammed),
  start_address_(0x00010000)
{
  node_.setParam("EtherCAT_Initialized", false);
  if (!interface_.empty())
    init();
  else
    ROS_DEBUG("No ethercat interface given. EthercatHardware will not be initialized");
}

EthercatHardware::~EthercatHardware()
{
  diagnostics_publisher_.stop();
  for (uint32_t i = 0; i < slaves_.size(); ++i)
  {
    EC_FixedStationAddress fsa(i + 1);
    EtherCAT_SlaveHandler *sh = ethercat_master_->get_slave_handler(fsa);
    if (sh)
      sh->to_state(EC_PREOP_STATE);
  }
  if (ni_)
  {
    close_socket(ni_);
  }
  delete[] buffers_;
  motor_publisher_.stop();
  delete oob_com_;
  delete ethercat_master_;
  delete m_router_;
  delete application_layer_;
}

bool EthercatHardware::setRouterToSlaveHandlers()
{
  unsigned int n_slaves = application_layer_->get_num_slaves();
  EtherCAT_SlaveHandler **slaves = application_layer_->get_slaves();
  for (unsigned int i = 0; i < n_slaves; ++i)
    slaves[i]->setRouter(m_router_);
  return true;
}

void EthercatHardware::changeState(EtherCAT_SlaveHandler *sh, EC_State new_state)
{
  unsigned product_code = sh->get_product_code();
  unsigned serial = sh->get_serial();
  uint32_t revision = sh->get_revision();
  unsigned slave = sh->get_station_address() - 1;

  if (!sh->to_state(new_state))
  {
    ROS_FATAL("Cannot goto state %d for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
              new_state,
              slave, product_code, product_code, serial, serial, revision, revision);
    if ((product_code == 0xbaddbadd) || (serial == 0xbaddbadd) || (revision == 0xbaddbadd))
      ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
    exit(EXIT_FAILURE);
  }
}

std::vector<EtherCAT_SlaveHandler> EthercatHardware::scanPort(const std::string& eth)
{
  std::vector<EtherCAT_SlaveHandler> detected_devices;
  int sock = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
  {
    int error = errno;
    ROS_FATAL("Couldn't open temp socket : %s", strerror(error));
    return detected_devices;
  }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, eth.c_str(), IFNAMSIZ);
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
  {
    int error = errno;
    ROS_FATAL("Cannot get interface_.c_str() flags for %s: %s", eth.c_str(),
              strerror(error));
    return detected_devices;
  }

  close(sock);
  sock = -1;

  if (!(ifr.ifr_flags & IFF_UP))
  {
    ROS_FATAL("Interface %s is not UP. Try : ifup %s", eth.c_str(), eth.c_str());
    return detected_devices;
  }
  if (!(ifr.ifr_flags & IFF_RUNNING))
  {
    ROS_FATAL("Interface %s is not RUNNING. Is cable plugged in and device powered?",
              eth.c_str());
    return detected_devices;
  }

  // Initialize network interface
  struct netif *ni(NULL);
  if ((ni = init_ec(eth.c_str())) == NULL)
  {
    ROS_FATAL_STREAM("Unable to initialize interface_: " << eth);
    return detected_devices;
  }
  EC_Logic logic_instance;
  EtherCAT_DataLinkLayer dll_instance;
  EtherCAT_PD_Buffer pd_buffer(&logic_instance, &dll_instance);
  boost::scoped_ptr<EtherCAT_AL> application_layer;
  boost::scoped_ptr<EtherCAT_Router> router;

  dll_instance.attach(ni);
  application_layer.reset(new EtherCAT_AL(&dll_instance, &logic_instance, &pd_buffer));
  router.reset(new EtherCAT_Router(application_layer.get(), &logic_instance, &dll_instance));

  unsigned int n_slaves = application_layer->get_num_slaves();
  EtherCAT_SlaveHandler **slaves = application_layer->get_slaves();
  for (unsigned int i = 0; i < n_slaves; ++i)
  {
      detected_devices.push_back(EtherCAT_SlaveHandler(*slaves[i]));
  }
  if (ni)
  {
    close_socket(ni);
  }
  return detected_devices;
}

void EthercatHardware::init()
{
  // open temporary socket to use with ioctl
  int sock = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
  {
    int error = errno;
    ROS_FATAL("Couldn't open temp socket : %s", strerror(error));
    sleep(1);
    exit(EXIT_FAILURE);
  }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ);
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
  {
    int error = errno;
    ROS_FATAL("Cannot get interface_.c_str() flags for %s: %s", interface_.c_str(),
              strerror(error));
    sleep(1);
    exit(EXIT_FAILURE);
  }

  close(sock);
  sock = -1;

  if (!(ifr.ifr_flags & IFF_UP))
  {
    ROS_FATAL("Interface %s is not UP. Try : ifup %s", interface_.c_str(), interface_.c_str());
    sleep(1);
    exit(EXIT_FAILURE);
  }
  if (!(ifr.ifr_flags & IFF_RUNNING))
  {
    ROS_FATAL("Interface %s is not RUNNING. Is cable plugged in and device powered?",
              interface_.c_str());
    sleep(1);
    exit(EXIT_FAILURE);
  }

  // Initialize network interface
  if ((ni_ = init_ec(interface_.c_str())) == NULL)
  {
    ROS_FATAL_STREAM("Unable to initialize interface_: " << interface_);
    sleep(1);
    exit(EXIT_FAILURE);
  }

  m_dll_instance_.attach(ni_);
  application_layer_ = new EtherCAT_AL(&m_dll_instance_, &m_logic_instance_, &pd_buffer_);
  m_router_ = new EtherCAT_Router(application_layer_, &m_logic_instance_, &m_dll_instance_);
  setRouterToSlaveHandlers();
  ethercat_master_ = new EtherCAT_Master(application_layer_, m_router_, &pd_buffer_, &m_logic_instance_, &m_dll_instance_);
  oob_com_ = new EthercatOobCom(ni_);

  int num_ethercat_devices_ = application_layer_->get_num_slaves();
  if (num_ethercat_devices_ == 0)
  {
    ROS_FATAL("Unable to locate any slaves");
    sleep(1);
    exit(EXIT_FAILURE);
  }

  // Size slaves vector to hold appropriate number of device pointers
  slaves_.resize(num_ethercat_devices_);

  // Make temporary list of slave handles
  std::vector<EtherCAT_SlaveHandler*> slave_handles;
  for (unsigned int slave = 0; slave < slaves_.size(); ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = ethercat_master_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      ROS_FATAL("Unable to get slave handler #%d", slave);
      sleep(1);
      exit(EXIT_FAILURE);
    }
    slave_handles.push_back(sh);
  }

  // Configure EtherCAT slaves

  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    unsigned slave = sh->get_station_address() - 1;
    if ((slaves_[slave] = configSlave(sh)) == NULL)
    {
      ROS_FATAL("Unable to configure slave #%d", slave);
      sleep(1);
      exit(EXIT_FAILURE);
    }
    buffer_size_ += slaves_[slave]->command_size_ + slaves_[slave]->status_size_;
  }

  // Configure any non-ethercat slaves (appends devices to slaves_ vector)
  loadNonEthercatDevices();

  // Move slave from INIT to PREOP

  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh, EC_PREOP_STATE);
  }

  // Move slave from PREOP to SAFEOP

  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh, EC_SAFEOP_STATE);
  }

  // Move slave from SAFEOP to OP
  // TODO : move to OP after initializing slave process data

  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh, EC_OP_STATE);
  }

  // Allocate buffers to send and receive commands
  buffers_ = new unsigned char[2 * buffer_size_];
  this_buffer_ = buffers_;
  prev_buffer_ = buffers_ + buffer_size_;

  // Make sure motors are disabled, also collect status data
  memset(this_buffer_, 0, 2 * buffer_size_);
  if (!txandrx_PD(buffer_size_, this_buffer_, 20))
  {
    ROS_FATAL("No communication with devices");
    sleep(1);
    exit(EXIT_FAILURE);
  }

  // prev_buffer should contain valid status data when update function is first used
  memcpy(prev_buffer_, this_buffer_, buffer_size_);

  last_published_ = ros::Time::now();

  // Initialize slaves
  //set<string> actuator_names;
  for (unsigned int slave = 0; slave < slaves_.size(); ++slave)
  {
    if (slaves_[slave]->initialize(hw_, allow_unprogrammed_) < 0)
    {
      EtherCAT_SlaveHandler *sh = slaves_[slave]->sh_;
      if (sh != NULL)
      {
        ROS_FATAL("Unable to initialize slave #%d, product code: %d, revision: %d, serial: %d",
                  slave,
                  sh->get_product_code(), sh->get_revision(), sh->get_serial());
        sleep(1);
      }
      else
      {
        ROS_FATAL("Unable to initialize slave #%d", slave);
      }
      exit(EXIT_FAILURE);
    }
  }

  { // Initialization is now complete. Reduce timeout of EtherCAT txandrx for better realtime performance
    // Allow timeout to be configured at program load time with rosparam.
    // This will allow tweaks for systems with different realtime performance
    static const int MAX_TIMEOUT = 100000; // 100ms = 100,000us
    static const int DEFAULT_TIMEOUT = 20000; // default to timeout to 20000us = 20ms
    int timeout;
    if (!node_.getParam("realtime_socket_timeout", timeout))
    {
      timeout = DEFAULT_TIMEOUT;
    }
    if ((timeout <= 1) || (timeout > MAX_TIMEOUT))
    {
      int old_timeout = timeout;
      timeout = std::max(1, std::min(MAX_TIMEOUT, timeout));
      ROS_WARN("Invalid timeout (%d) for socket, using %d", old_timeout, timeout);
    }
    if (set_socket_timeout(ni_, timeout))
    {
      ROS_FATAL("Error setting socket timeout to %d", timeout);
      sleep(1);
      exit(EXIT_FAILURE);
    }
    timeout_ = timeout;

    // When packet containing process data is does not return after a given timeout, it is
    // assumed to be dropped and the process data will automatically get re-sent.
    // After a number of retries, the driver will halt motors as a safety precaution.
    //
    // The following code allows the number of process data retries to be changed with a rosparam.
    // This is needed because lowering the txandrx timeout makes it more likely that a
    // performance glitch in network or OS causes will cause the motors to halt.
    //
    // If number of retries is not specified, use a formula that allows 100ms of dropped packets
    int max_pd_retries = MAX_TIMEOUT / timeout; // timeout is in nanoseconds : 20msec = 20000usec
    static const int MAX_RETRIES = 50, MIN_RETRIES = 1;
    node_.getParam("max_pd_retries", max_pd_retries);
    // Make sure motor halt due to dropped packet takes less than 1/10 of a second
    if ((max_pd_retries * timeout) > (MAX_TIMEOUT))
    {
      max_pd_retries = MAX_TIMEOUT / timeout;
      ROS_WARN("Max PD retries is too large for given timeout.  Limiting value to %d",
               max_pd_retries);
    }
    if ((max_pd_retries < MIN_RETRIES) || (max_pd_retries > MAX_RETRIES))
    {
      max_pd_retries = std::max(MIN_RETRIES, std::min(MAX_RETRIES, max_pd_retries));
      ROS_WARN("Limiting max PD retries to %d", max_pd_retries);
    }
    max_pd_retries = std::max(MIN_RETRIES, std::min(MAX_RETRIES, max_pd_retries));
    max_pd_retries_ = max_pd_retries;
  }

  diagnostics_publisher_.initialize(interface_, buffer_size_, slaves_, num_ethercat_devices_,
                                    timeout_,
                                    max_pd_retries_);

  node_.setParam("EtherCAT_Initialized", true);
}

EthercatHardwareDiagnosticsPublisher::EthercatHardwareDiagnosticsPublisher(ros::NodeHandle &node) :
  node_(node),
  diagnostics_ready_(false),
  publisher_(node_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1)),
  diagnostics_buffer_(NULL),
  last_dropped_packet_count_(0),
  last_dropped_packet_time_(0)
{
}

EthercatHardwareDiagnosticsPublisher::~EthercatHardwareDiagnosticsPublisher()
{
  delete[] diagnostics_buffer_;
}

void EthercatHardwareDiagnosticsPublisher::initialize(const string &interface, unsigned int buffer_size,
                                                      const std::vector<boost::shared_ptr<EthercatDevice> > &slaves,
                                                      unsigned int num_ethercat_devices,
                                                      unsigned timeout,
                                                      unsigned max_pd_retries)
{
  interface_ = interface;
  buffer_size_ = buffer_size;
  slaves_ = slaves;
  num_ethercat_devices_ = num_ethercat_devices;
  timeout_ = timeout;
  max_pd_retries_ = max_pd_retries;

  diagnostics_buffer_ = new unsigned char[buffer_size_];

  // Initialize diagnostic data structures
  diagnostic_array_.status.reserve(slaves_.size() + 1);
  values_.reserve(10);

  ethernet_interface_info_.initialize(interface);

  diagnostics_thread_ = boost::thread(boost::bind(&EthercatHardwareDiagnosticsPublisher::diagnosticsThreadFunc, this));
}

void EthercatHardwareDiagnosticsPublisher::publish(const unsigned char *buffer,
                                                   const EthercatHardwareDiagnostics &diagnostics)
{
  boost::try_to_lock_t try_lock;
  boost::unique_lock<boost::mutex> lock(diagnostics_mutex_, try_lock);
  if (lock.owns_lock())
  {
    // Make copies of diagnostic data for diagnostic thread
    memcpy(diagnostics_buffer_, buffer, buffer_size_);
    diagnostics_ = diagnostics;
    // Trigger diagnostics publish thread
    diagnostics_ready_ = true;
    diagnostics_cond_.notify_one();
  }
}

void EthercatHardwareDiagnosticsPublisher::stop()
{
  diagnostics_thread_.interrupt();
  diagnostics_thread_.join();
  publisher_.shutdown();
}

void EthercatHardwareDiagnosticsPublisher::diagnosticsThreadFunc()
{
  try
  {
    while (1)
    {
      boost::unique_lock<boost::mutex> lock(diagnostics_mutex_);
      while (!diagnostics_ready_)
      {
        diagnostics_cond_.wait(lock);
      }
      diagnostics_ready_ = false;
      publishDiagnostics();
    }
  }
  catch (boost::thread_interrupted const&)
  {
    return;
  }
}

void EthercatHardwareDiagnosticsPublisher::timingInformation(diagnostic_updater::DiagnosticStatusWrapper &status,
                                                             const string &key,
                                                             const accumulator_set<double, stats<tag::max, tag::mean> > &acc,
                                                             double max)
{
  status.addf(key + " Avg (us)", "%5.4f", extract_result<tag::mean>(acc) * 1e6); // Average over last 1 second
  status.addf(key + " 1 Sec Max (us)", "%5.4f", extract_result<tag::max>(acc) * 1e6); // Max over last 1 second
  status.addf(key + " Max (us)", "%5.4f", max * 1e6); // Max since start
}

void EthercatHardwareDiagnosticsPublisher::publishDiagnostics()
{
  ros::Time now(ros::Time::now());

  // Publish status of EtherCAT master
  status_.clearSummary();
  status_.clear();

  status_.name = "EtherCAT Master";
//  if (diagnostics_.motors_halted_)
//  {
//    std::ostringstream desc;
//    desc << "Motors halted";
//    if (diagnostics_.halt_after_reset_)
//    {
//      desc << " soon after reset";
//    }
//    desc << " (" << diagnostics_.motors_halted_reason_ << ")";
//    status_.summary(status_.ERROR, desc.str());
//  }
//  else
//  {
  status_.summary(status_.OK, "OK");
//  }

  if (diagnostics_.pd_error_)
  {
    status_.mergeSummary(status_.ERROR, "Error sending proccess data");
  }

  //status_.add("Motors halted", diagnostics_.motors_halted_ ? "true" : "false");
  status_.addf("EtherCAT devices (expected)", "%d", num_ethercat_devices_);
  status_.addf("EtherCAT devices (current)", "%d", diagnostics_.device_count_);
  ethernet_interface_info_.publishDiagnostics(status_);
  //status_.addf("Reset state", "%d", reset_state_);

  status_.addf("Timeout (us)", "%d", timeout_);
  status_.addf("Max PD Retries", "%d", max_pd_retries_);

  // Produce warning if number of devices changed after device initialization
  if (num_ethercat_devices_ != diagnostics_.device_count_)
  {
    status_.mergeSummary(status_.WARN, "Number of EtherCAT devices changed");
  }

  timingInformation(status_, "Roundtrip time", diagnostics_.txandrx_acc_,
                    diagnostics_.max_txandrx_);
  if (diagnostics_.collect_extra_timing_)
  {
    timingInformation(status_, "Pack command time", diagnostics_.pack_command_acc_,
                      diagnostics_.max_pack_command_);
    timingInformation(status_, "Unpack state time", diagnostics_.unpack_state_acc_,
                      diagnostics_.max_unpack_state_);
    timingInformation(status_, "Publish time", diagnostics_.publish_acc_,
                      diagnostics_.max_publish_);
  }

  status_.addf("EtherCAT Process Data txandrx errors", "%d", diagnostics_.txandrx_errors_);

  status_.addf("Reset motors service count", "%d", diagnostics_.reset_motors_service_count_);
  status_.addf("Halt motors service count", "%d", diagnostics_.halt_motors_service_count_);
  status_.addf("Halt motors error count", "%d", diagnostics_.halt_motors_error_count_);

  { // Publish ethercat network interface counters
    const struct netif_counters *c = &diagnostics_.counters_;
    status_.add("Input Thread", (diagnostics_.input_thread_is_stopped_ ? "Stopped" : "Running"));
    status_.addf("Sent Packets", "%llu", (unsigned long long) c->sent);
    status_.addf("Received Packets", "%llu", (unsigned long long) c->received);
    status_.addf("Collected Packets", "%llu", (unsigned long long) c->collected);
    status_.addf("Dropped Packets", "%llu", (unsigned long long) c->dropped);
    status_.addf("TX Errors", "%llu", (unsigned long long) c->tx_error);
    status_.addf("TX Network Down", "%llu", (unsigned long long) c->tx_net_down);
    status_.addf("TX Would Block", "%llu", (unsigned long long) c->tx_would_block);
    status_.addf("TX No Buffers", "%llu", (unsigned long long) c->tx_no_bufs);
    status_.addf("TX Queue Full", "%llu", (unsigned long long) c->tx_full);
    status_.addf("RX Runt Packet", "%llu", (unsigned long long) c->rx_runt_pkt);
    status_.addf("RX Not EtherCAT", "%llu", (unsigned long long) c->rx_not_ecat);
    status_.addf("RX Other EML", "%llu", (unsigned long long) c->rx_other_eml);
    status_.addf("RX Bad Index", "%llu", (unsigned long long) c->rx_bad_index);
    status_.addf("RX Bad Sequence", "%llu", (unsigned long long) c->rx_bad_seqnum);
    status_.addf("RX Duplicate Sequence", "%llu", (unsigned long long) c->rx_dup_seqnum);
    status_.addf("RX Duplicate Packet", "%llu", (unsigned long long) c->rx_dup_pkt);
    status_.addf("RX Bad Order", "%llu", (unsigned long long) c->rx_bad_order);
    status_.addf("RX Late Packet", "%llu", (unsigned long long) c->rx_late_pkt);
    status_.addf("RX Late Packet RTT", "%llu", (unsigned long long) c->rx_late_pkt_rtt_us);

    double rx_late_pkt_rtt_us_avg = 0.0;
    if (c->rx_late_pkt > 0)
    {
      rx_late_pkt_rtt_us_avg = ((double) c->rx_late_pkt_rtt_us_sum) / ((double) c->rx_late_pkt);
    }
    status_.addf("RX Late Packet Avg RTT", "%f", rx_late_pkt_rtt_us_avg);

    // Check for newly dropped packets
    if (c->dropped > last_dropped_packet_count_)
    {
      last_dropped_packet_time_ = now;
      last_dropped_packet_count_ = c->dropped;
    }
  }

  // Create error message if packet has been dropped recently
  if ((last_dropped_packet_count_ > 0)
      && ((now - last_dropped_packet_time_).toSec() < dropped_packet_warning_hold_time_))
  {
    status_.mergeSummaryf(status_.WARN, "Dropped packets in last %d seconds",
                          dropped_packet_warning_hold_time_);
  }

  diagnostic_array_.status.clear();
  diagnostic_array_.status.push_back(status_);

  // Also, collect diagnostic statuses of all EtherCAT device
  unsigned char *current = diagnostics_buffer_;
  for (unsigned int s = 0; s < slaves_.size(); ++s)
  {
    slaves_[s]->multiDiagnostics(diagnostic_array_.status, current);
    current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
  }

  // Publish status of each EtherCAT device
  diagnostic_array_.header.stamp = ros::Time::now();
  publisher_.publish(diagnostic_array_);
}

void EthercatHardware::update(bool reset, bool halt)
{
  // Update current time
  ros::Time update_start_time(ros::Time::now());

  unsigned char *this_buffer, *prev_buffer;

  // Convert HW Interface commands to MCB-specific buffers
  this_buffer = this_buffer_;

  if (halt)
  {
    ++diagnostics_.halt_motors_service_count_;
    haltMotors(false /*no error*/, "service request");
  }

  // Resetting devices should clear device errors and release devices from halt.
  // To reduce load on power system, release devices from halt, one at a time
  const unsigned CYCLES_PER_HALT_RELEASE = 2; // Wait two cycles between releasing each device
  if (reset)
  {
    ++diagnostics_.reset_motors_service_count_;
    reset_state_ = CYCLES_PER_HALT_RELEASE * slaves_.size() + 5;
    last_reset_ = update_start_time;
    diagnostics_.halt_after_reset_ = false;
  }
  bool reset_devices = reset_state_ == CYCLES_PER_HALT_RELEASE * slaves_.size() + 3;
  if (reset_devices)
  {
    halt_motors_ = false;
    diagnostics_.motors_halted_ = false;
    diagnostics_.motors_halted_reason_ = "";
    diagnostics_.resetMaxTiming();
    diagnostics_.pd_error_ = false;
  }

  for (unsigned int s = 0; s < slaves_.size(); ++s)
  {
    // Pack the command structures into the EtherCAT buffer
    // Disable the motor if they are halted or coming out of reset
    bool halt_device = halt_motors_ || ((s * CYCLES_PER_HALT_RELEASE + 1) < reset_state_);
    slaves_[s]->packCommand(this_buffer, halt_device, reset_devices);
    this_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
  }

  // Transmit process data
  ros::Time txandrx_start_time(ros::Time::now()); // Also end time for pack_command_stage
  diagnostics_.pack_command_acc_((txandrx_start_time - update_start_time).toSec());

  // Send/receive device process data
  bool success = txandrx_PD(buffer_size_, this_buffer_, max_pd_retries_);

  ros::Time txandrx_end_time(ros::Time::now()); // Also start unpack_state
  diagnostics_.txandrx_acc_((txandrx_end_time - txandrx_start_time).toSec());

  if (!success)
  {
    // If process data didn't get sent after multiple retries, stop motors
    haltMotors(true /*error*/, "communication error");
    diagnostics_.pd_error_ = true;
  }
  else
  {
    // Convert status back to HW Interface
    this_buffer = this_buffer_;
    prev_buffer = prev_buffer_;
    for (unsigned int s = 0; s < slaves_.size(); ++s)
    {
      if (!slaves_[s]->unpackState(this_buffer, prev_buffer) && !reset_devices)
      {
        haltMotors(true /*error*/, "device error");
      }
      this_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      prev_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
    }

    if (reset_state_)
      --reset_state_;

    unsigned char *tmp = this_buffer_;
    this_buffer_ = prev_buffer_;
    prev_buffer_ = tmp;
  }

  ros::Time unpack_end_time;
  if (diagnostics_.collect_extra_timing_)
  {
    unpack_end_time = ros::Time::now(); // also start of publish time
    diagnostics_.unpack_state_acc_((unpack_end_time - txandrx_end_time).toSec());
  }

  if ((update_start_time - last_published_) > ros::Duration(1.0))
  {
    last_published_ = update_start_time;
    publishDiagnostics();
    motor_publisher_.lock();
    motor_publisher_.msg_.data = halt_motors_;
    motor_publisher_.unlockAndPublish();
  }

  if (diagnostics_.collect_extra_timing_)
  {
    ros::Time publish_end_time(ros::Time::now());
    diagnostics_.publish_acc_((publish_end_time - unpack_end_time).toSec());
  }
}

void EthercatHardware::haltMotors(bool error, const char* reason)
{
  if (!halt_motors_)
  {
    // wasn't already halted
    motor_publisher_.lock();
    motor_publisher_.msg_.data = halt_motors_;
    motor_publisher_.unlockAndPublish();

    diagnostics_.motors_halted_reason_ = reason;
    if (error)
    {
      ++diagnostics_.halt_motors_error_count_;
      if ((ros::Time::now() - last_reset_) < ros::Duration(0.5))
      {
        // halted soon after reset
        diagnostics_.halt_after_reset_ = true;
      }
    }
  }

  diagnostics_.motors_halted_ = true;
  halt_motors_ = true;
}

void EthercatHardware::updateAccMax(double &max,
                                    const accumulator_set<double, stats<tag::max, tag::mean> > &acc)
{
  max = std::max(max, extract_result<tag::max>(acc));
}

void EthercatHardware::publishDiagnostics()
{
  // Update max timing values
  updateAccMax(diagnostics_.max_pack_command_, diagnostics_.pack_command_acc_);
  updateAccMax(diagnostics_.max_txandrx_, diagnostics_.txandrx_acc_);
  updateAccMax(diagnostics_.max_unpack_state_, diagnostics_.unpack_state_acc_);
  updateAccMax(diagnostics_.max_publish_, diagnostics_.publish_acc_);

  // Grab stats and counters from input thread
  diagnostics_.counters_ = ni_->counters;
  diagnostics_.input_thread_is_stopped_ = bool(ni_->is_stopped);

  diagnostics_.motors_halted_ = halt_motors_;

  // Pass diagnostic data to publisher thread
  diagnostics_publisher_.publish(this_buffer_, diagnostics_);

  // Clear statistics accumulators
  static accumulator_set<double, stats<tag::max, tag::mean> > blank;
  diagnostics_.pack_command_acc_ = blank;
  diagnostics_.txandrx_acc_ = blank;
  diagnostics_.unpack_state_acc_ = blank;
  diagnostics_.publish_acc_ = blank;
}

boost::shared_ptr<EthercatDevice>
EthercatHardware::configSlave(EtherCAT_SlaveHandler *sh)
{
  boost::shared_ptr<EthercatDevice> p;
  unsigned product_code = sh->get_product_code();
  unsigned serial = sh->get_serial();
  uint32_t revision = sh->get_revision();
  unsigned slave = sh->get_station_address() - 1;

  // The point of this code to find a class whose name matches the EtherCAT
  // product ID for a given device.
  // Thus device plugins would register themselves with PLUGINLIB_EXPORT_CLASS
  //
  //    PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)
  //
  // and in the plugin.xml, specify name="" inside the <class> tag:
  //
  //    <class name="package/serial"
  //           base_class_type="ethercat_hardware::EthercatDevice" />
  //
  //
  // For the WG05 driver (productID = 6805005), this statement would look
  // something like:
  //
  //    PLUGINLIB_EXPORT_CLASS(WG05, EthercatDevice)
  //
  // and in the plugin.xml:
  //
  //    <class name="ethercat_hardware/6805005" type="WG05"
  //           base_class_type="EthercatDevice">
  //      <description>
  //        WG05 - Generic Motor Control Board
  //      </description>
  //    </class>
  //
  //
  // Unfortunately, we don't know which ROS package that a particular driver is defined in.
  // To account for this, we search through the list of class names, one-by-one and find string where
  // last part of string matches product ID of device.
  stringstream class_name_regex_str;
  class_name_regex_str << "(.*/)?" << product_code;
  boost::regex class_name_regex(class_name_regex_str.str(), boost::regex::extended);

  std::vector<std::string> classes = device_loader_.getDeclaredClasses();
  std::string matching_class_name;

  BOOST_FOREACH(const std::string &class_name, classes)
  {
    if (regex_match(class_name, class_name_regex))
    {
      if (matching_class_name.size() != 0)
      {
        ROS_ERROR("Found more than 1 EtherCAT driver for device with product code : %d", product_code);
        ROS_ERROR("First class name = '%s'.  Second class name = '%s'",
                  class_name.c_str(), matching_class_name.c_str());
      }
      matching_class_name = class_name;
    }
  }

  if (matching_class_name.size() != 0)
  {
    ROS_WARN("Using device '%s' with product code %d",
             device_loader_.getClassDescription(matching_class_name).c_str(),
             product_code);
    try
    {
      p = device_loader_.createInstance(matching_class_name);
    }
    catch (pluginlib::LibraryLoadException &e)
    {
      p.reset();
      ROS_FATAL("Unable to load plugin for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                slave,
                product_code, product_code, serial, serial, revision, revision);
      ROS_FATAL("%s", e.what());
    }
  }
  else
  {
    if ((product_code == 0xbaddbadd) || (serial == 0xbaddbadd) || (revision == 0xbaddbadd))
    {
      ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
      ROS_FATAL("Perhaps you should power-cycle the MCBs");
    }
    else
    {
      ROS_ERROR("Unable to load plugin for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                slave,
                product_code, product_code, serial, serial, revision, revision);
      ROS_ERROR("Possible classes:");

      BOOST_FOREACH(const std::string &class_name, classes)
      {
        ROS_ERROR("  %s", class_name.c_str());
      }

      // TODO, use default plugin for ethercat devices that have no driver.
      // This way, the EtherCAT chain still works.
    }
  }

  if (p != NULL)
  {
    p->construct(sh, start_address_);
  }

  return p;
}

boost::shared_ptr<EthercatDevice>
EthercatHardware::configNonEthercatDevice(const std::string &name, const std::string &type)
{
  boost::shared_ptr<EthercatDevice> p;
  try
  {
    p = device_loader_.createInstance(type);
  }
  catch (pluginlib::LibraryLoadException &e)
  {
    p.reset();
    ROS_FATAL("Unable to load plugin for non-EtherCAT device '%s' with type: %s : %s",
              name.c_str(), type.c_str(), e.what());
  }
  if (p)
  {
    ROS_INFO("Creating non-EtherCAT device '%s' of type '%s'", name.c_str(), type.c_str());
    ros::NodeHandle nh(node_, "non_ethercat_devices/" + name);
    p->construct(nh);
  }
  return p;
}

// Do this to get access to Struct (std::map) element of XmlRpcValue

class MyXmlRpcValue : public XmlRpc::XmlRpcValue
{
public:

  MyXmlRpcValue(XmlRpc::XmlRpcValue &value) :
    XmlRpc::XmlRpcValue(value)
  {
  }

  XmlRpcValue::ValueStruct &getMap()
  {
    return *_value.asStruct;
  }
};

void EthercatHardware::loadNonEthercatDevices()
{
  // non-EtherCAT device drivers are descibed via struct named "non_ethercat_devices"
  // Each element of "non_ethercat_devices" must be a struct that contains a "type" field.
  // The element struct can also contain other elements that are used as configuration parameters
  // for the device plugin.
  // Example:
  //   non_ethercat_devices:
  //     netft1:
  //       type: NetFT
  //       address: 192.168.1.1
  //       publish_period: 0.01
  //     netft2:
  //       type: NetFT
  //       address: 192.168.1.2
  //       publish_period: 0.01
  //     dummy_device:
  //       type: Dummy
  //       msg: "This a dummy device"
  //
  //   The name of the device is used to create a ros::NodeHandle that is
  //    that is passed to construct function of device class.
  //   NOTE: construct function is not the same as C++ construtor
  if (!node_.hasParam("non_ethercat_devices"))
  {
    // It is perfectly fine if there is no list of non-ethercat devices
    return;
  }

  XmlRpc::XmlRpcValue devices;
  node_.getParam("non_ethercat_devices", devices);
  if (devices.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("Rosparam 'non_ethercat_devices' is not a struct type");
    return;
  }

  MyXmlRpcValue my_devices(devices);
  typedef XmlRpc::XmlRpcValue::ValueStruct::value_type map_item_t;

  BOOST_FOREACH(map_item_t &item, my_devices.getMap())
  {
    const std::string & name(item.first);
    XmlRpc::XmlRpcValue & device_info(item.second);

    if (device_info.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("non_ethercat_devices/%s is not a struct type", name.c_str());
      continue;
    }

    if (!device_info.hasMember("type"))
    {
      ROS_ERROR("non_ethercat_devices/%s 'type' element", name.c_str());
      continue;
    }

    std::string type(static_cast<std::string> (device_info["type"]));

    boost::shared_ptr<EthercatDevice> new_device =
      configNonEthercatDevice(name, type);
    if (new_device != NULL)
    {
      slaves_.push_back(new_device);
    }
  }
}

void EthercatHardware::collectDiagnostics()
{
  if (NULL == oob_com_)
    return;

  { // Count number of devices
    unsigned char p[1];
    uint16_t length = sizeof (p);

    // Build read telegram, use slave position
    APRD_Telegram status(m_logic_instance_.get_idx(), // Index
                         0, // Slave position on ethercat chain (auto increment address)
                         0, // ESC physical memory address (start address)
                         m_logic_instance_.get_wkc(), // Working counter
                         length, // Data Length,
                         p); // Buffer to put read result into

    // Put read telegram in ros_ethercat_eml/ethernet frame
    EC_Ethernet_Frame frame(&status);
    oob_com_->txandrx(&frame);

    // Worry about locking for single value?
    diagnostics_.device_count_ = status.get_adp();
  }

  for (unsigned i = 0; i < slaves_.size(); ++i)
  {
    boost::shared_ptr<EthercatDevice> d(slaves_[i]);
    d->collectDiagnostics(oob_com_);
  }
}

// Prints (error) counter information of network interface driver

void EthercatHardware::printCounters(std::ostream &os)
{
  const struct netif_counters & c(ni_->counters);
  os << "netif counters :" << endl
    << " sent          = " << c.sent << endl
    << " received      = " << c.received << endl
    << " collected     = " << c.collected << endl
    << " dropped       = " << c.dropped << endl
    << " tx_error      = " << c.tx_error << endl
    << " tx_net_down   = " << c.tx_net_down << endl
    << " tx_would_block= " << c.tx_would_block << endl
    << " tx_no_bufs    = " << c.tx_no_bufs << endl
    << " tx_full       = " << c.tx_full << endl
    << " rx_runt_pkt   = " << c.rx_runt_pkt << endl
    << " rx_not_ecat   = " << c.rx_not_ecat << endl
    << " rx_other_eml  = " << c.rx_other_eml << endl
    << " rx_bad_index  = " << c.rx_bad_index << endl
    << " rx_bad_seqnum = " << c.rx_bad_seqnum << endl
    << " rx_dup_seqnum = " << c.rx_dup_seqnum << endl
    << " rx_dup_pkt    = " << c.rx_dup_pkt << endl
    << " rx_bad_order  = " << c.rx_bad_order << endl
    << " rx_late_pkt   = " << c.rx_late_pkt << endl;
}

bool EthercatHardware::txandrx_PD(unsigned buffer_size, unsigned char* buffer, unsigned tries)
{
  // Try multiple times to get process data to device
  bool success = false;
  for (unsigned i = 0; i < tries && !success; ++i)
  {
    // Try transmitting process data
    success = ethercat_master_->txandrx_PD(buffer_size_, this_buffer_);
    if (!success)
    {
      ++diagnostics_.txandrx_errors_;
    }
    // Transmit new OOB data
    oob_com_->tx();
  }
  return success;
}

bool EthercatHardware::publishTrace(int position, const string &reason, unsigned level,
                                    unsigned delay)
{
  if (position >= (int) slaves_.size())
  {
    ROS_WARN("Invalid device position %d.  Use 0-%d, or -1.", position, int(slaves_.size()) - 1);
    return false;
  }

  if (level > 2)
  {
    ROS_WARN("Invalid level : %d.  Using level=2 (ERROR).", level);
    level = 2;
  }

  string new_reason("Manually triggered : " + reason);

  bool retval = false;
  if (position < 0)
  {
    for (unsigned i = 0; i < slaves_.size(); ++i)
    {
      if (slaves_[i]->publishTrace(new_reason, level, delay))
      {
        retval = true;
      }
    }
  }
  else
  {
    retval = slaves_[position]->publishTrace(new_reason, level, delay);
    if (!retval)
    {
      ROS_WARN("Device %d does not support publishing trace", position);
    }
  }
  return retval;
}
