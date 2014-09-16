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

#ifndef ETHERCAT_DEVICE_H
#define ETHERCAT_DEVICE_H

#include <vector>

#include <ros_ethercat_eml/ethercat_defs.h>
#include <ros_ethercat_eml/ethercat_slave_handler.h>
#include <ros_ethercat_eml/ethercat_device_addressed_telegram.h>
#include <ros_ethercat_eml/ethercat_logical_addressed_telegram.h>
#include <ros_ethercat_eml/ethercat_frame.h>

#include <hardware_interface/hardware_interface.h>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <ros_ethercat_hardware/ethercat_com.h>

#include <pluginlib/class_list_macros.h>

using namespace std;

struct et1x00_error_counters
{

  struct
  {
    uint8_t invalid_frame;
    uint8_t rx_error;
  } __attribute__((__packed__)) port[4];
  uint8_t forwarded_rx_error[4];
  uint8_t epu_error;
  uint8_t pdi_error;
  uint8_t res[2];
  uint8_t lost_link[4];
  static const uint16_t BASE_ADDR = 0x300;
  bool isGreaterThan(unsigned value) const;
  bool isGreaterThan(const et1x00_error_counters &value) const;
  void zero();
} __attribute__((__packed__));

struct et1x00_dl_status
{
  uint16_t status;
  bool hasLink(unsigned port);
  bool isClosed(unsigned port);
  bool hasCommunication(unsigned port);
  static const uint16_t BASE_ADDR = 0x110;
} __attribute__((__packed__));

struct EthercatPortDiagnostics
{
  EthercatPortDiagnostics();
  void zeroTotals();
  bool hasLink;
  bool isClosed;
  bool hasCommunication;
  uint64_t rxErrorTotal;
  uint64_t invalidFrameTotal;
  uint64_t forwardedRxErrorTotal;
  uint64_t lostLinkTotal;
};

struct EthercatDeviceDiagnostics
{
public:
  EthercatDeviceDiagnostics();

  // Collects diagnostic data from specific ethercat slave, and updates object state
  //
  // com  EtherCAT communication object is used send/recv packets to/from ethercat chain.
  // sh   slaveHandler of device to collect Diagnostics from
  // prev previously collected diagnostics (can be pointer to this object)
  //
  // collectDiagnotics will send/receive multiple packets, and may considerable amount of time complete.
  //
  void collect(EthercatCom *com, EtherCAT_SlaveHandler *sh);

  // Puts previously diagnostic collected diagnostic state to DiagnosticStatus object
  //
  // d         DiagnositcState to add diagnostics to.
  // numPorts  Number of ports device is supposed to have.  4 is max, 1 is min.
  void publish(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned numPorts = 4) const;

protected:
  void zeroTotals();
  void accumulate(const et1x00_error_counters &next, const et1x00_error_counters &prev);
  uint64_t pdiErrorTotal_;
  uint64_t epuErrorTotal_;
  EthercatPortDiagnostics portDiagnostics_[4];
  unsigned nodeAddress_;
  et1x00_error_counters errorCountersPrev_;
  bool errorCountersMayBeCleared_;

  bool diagnosticsFirst_;
  bool diagnosticsValid_;
  bool resetDetected_;
  int devicesRespondingToNodeAddress_;
};

class EthercatDevice
{
public:
  //!< Construct EtherCAT device
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);

  //!< Construct non-EtherCAT device
  virtual void construct(ros::NodeHandle &nh)
  {
  }

  EthercatDevice();
  virtual ~EthercatDevice()
  {
    delete sh_->get_fmmu_config();
    delete sh_->get_pd_config();
  }
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed = true)
  {
    return 0;
  }
  /**
   * \param reset  when asserted this will clear diagnostic error conditions device safety disable
   * \param halt   while asserted will disable actuator, usually by disabling H-bridge
   */
  virtual void packCommand(unsigned char *buffer, bool halt, bool reset)
  {
  }
  virtual bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
  {
    return true;
  }

  /**
   * \brief For EtherCAT devices that publish more than one EtherCAT Status message.
   * If sub-class implements multiDiagnostics() then diagnostics() is not used.
   * \param vec     Vector of diagnostics status messages. Slave appends one or more new diagnostic status'.
   * \param buffer  Pointer to slave process data.\
   */
  virtual void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                unsigned char *buffer);

  /**
   * \brief For EtherCAT device that only publish one EtherCAT Status message.
   * If sub-class implements multiDiagnostics() then diagnostics() is not used.
   * \param d       Diagnostics status wrapper.
   * \param buffer  Pointer to slave process data.
   */
  virtual void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  /**
   * \brief Adds diagnostic information for EtherCAT ports.
   * \param d       EtherCAT port diagnostics information will be appended.
   * \param buffer  Number of communication ports slave has.
   */
  void ethercatDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned numPorts);

  virtual void collectDiagnostics(EthercatCom *com);
  /**
   * \brief Asks device to publish (motor) trace. Only works for devices that support it.
   * \param reason Message to put in trace as reason.
   * \param level Level to put in trace (aka ERROR=2, WARN=1, OK=0)
   * \param delay Publish trace after delay cyles.  For 1kHz realtime loop 1cycle = 1ms.
   * \return Return true if device support publishing trace.  False, if not.
   */
  virtual bool publishTrace(const string &reason, unsigned level, unsigned delay)
  {
    return false;
  }

  enum AddrMode
  {
    FIXED_ADDR = 0, POSITIONAL_ADDR = 1
  };

  /*!
   * \brief Write data to device ESC.
   */
  static int writeData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address,
                       void const* buffer, uint16_t length, AddrMode addrMode = FIXED_ADDR);
  inline int writeData(EthercatCom *com, uint16_t address, void const* buffer, uint16_t length,
                       AddrMode addrMode)
  {
    return writeData(com, sh_, address, buffer, length, addrMode);
  }

  /*!
   * \brief Read data from device ESC.
   */
  static int readData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address, void *buffer,
                      uint16_t length, AddrMode addrMode = FIXED_ADDR);
  inline int readData(EthercatCom *com, uint16_t address, void *buffer, uint16_t length,
                      AddrMode addrMode)
  {
    return readData(com, sh_, address, buffer, length, addrMode);
  }

  /*!
   * \brief Read then write data to ESC.
   */
  static int readWriteData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address,
                           void *buffer, uint16_t length, AddrMode addrMode = FIXED_ADDR);
  inline int readWriteData(EthercatCom *com, uint16_t address, void *buffer, uint16_t length,
                           AddrMode addrMode)
  {
    return readWriteData(com, sh_, address, buffer, length, addrMode);
  }

  bool use_ros_;

  EtherCAT_SlaveHandler *sh_;
  unsigned int command_size_;
  unsigned int status_size_;

  // The device diagnostics are collected with a non-readtime thread that calls collectDiagnostics()
  // The device published from the realtime loop by indirectly invoking ethercatDiagnostics()
  // To avoid blocking of the realtime thread (for long) a double buffer is used the
  // The publisher thread will lock newDiagnosticsIndex when publishing data.
  // The collection thread will lock deviceDiagnostics when updating deviceDiagnostics
  // The collection thread will also lock newDiagnosticsIndex at end of update, just before swapping buffers.
  unsigned newDiagnosticsIndex_;
  pthread_mutex_t newDiagnosticsIndexLock_;
  EthercatDeviceDiagnostics deviceDiagnostics[2];
  pthread_mutex_t diagnosticsLock_;

  // Keep diagnostics status as cache.  Avoids a lot of construction/destruction of status object.
  diagnostic_updater::DiagnosticStatusWrapper diagnostic_status_;
};

#endif /* ETHERCAT_DEVICE_H */
