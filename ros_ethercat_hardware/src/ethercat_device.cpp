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

#include "ros_ethercat_hardware/ethercat_device.h"

#include <tinyxml.h>

#include <iomanip>

bool et1x00_error_counters::isGreaterThan(unsigned value) const
{
  if ((pdi_error > value) || (epu_error > value))
  {
    return true;
  }

  for (unsigned i = 0; i < 4; ++i)
  {
    if ((port[i].rx_error > value) ||
        (forwarded_rx_error[i] > value) ||
        (lost_link[i] > value) ||
        (port[i].invalid_frame > value))
    {
      return true;
    }
  }
  return false;
}

bool et1x00_error_counters::isGreaterThan(const et1x00_error_counters &v) const
{
  if ((pdi_error > v.pdi_error) || (epu_error > v.epu_error))
  {
    return true;
  }

  for (unsigned i = 0; i < 4; ++i)
  {
    if ((port[i].rx_error > v.port[i].rx_error) ||
        (forwarded_rx_error[i] > v.forwarded_rx_error[i]) ||
        (lost_link[i] > v.lost_link[i]) ||
        (port[i].invalid_frame > v.port[i].invalid_frame))
    {
      return true;
    }
  }
  return false;
}

bool et1x00_dl_status::hasLink(unsigned port)
{
  assert(port < 4);
  return status & (1 << (4 + port));
}

bool et1x00_dl_status::hasCommunication(unsigned port)
{
  assert(port < 4);
  return status & (1 << (9 + port * 2));
}

bool et1x00_dl_status::isClosed(unsigned port)
{
  assert(port < 4);
  return status & (1 << (8 + port * 2));
}

void et1x00_error_counters::zero()
{
  memset(this, 0, sizeof (et1x00_error_counters));
}

EthercatPortDiagnostics::EthercatPortDiagnostics() :
  hasLink(false),
  isClosed(false),
  hasCommunication(false)
{
  zeroTotals();
}

void EthercatPortDiagnostics::zeroTotals()
{
  rxErrorTotal = 0;
  invalidFrameTotal = 0;
  forwardedRxErrorTotal = 0;
  lostLinkTotal = 0;
}

EthercatDeviceDiagnostics::EthercatDeviceDiagnostics() :
  errorCountersMayBeCleared_(false),
  diagnosticsFirst_(true),
  diagnosticsValid_(false),
  resetDetected_(false),
  devicesRespondingToNodeAddress_(-1)
{
  zeroTotals();
  errorCountersPrev_.zero();
}

void EthercatDeviceDiagnostics::zeroTotals()
{
  pdiErrorTotal_ = 0;
  epuErrorTotal_ = 0;
  portDiagnostics_[0].zeroTotals();
  portDiagnostics_[1].zeroTotals();
  portDiagnostics_[2].zeroTotals();
  portDiagnostics_[3].zeroTotals();
}

void EthercatDeviceDiagnostics::accumulate(const et1x00_error_counters &n, const et1x00_error_counters &p)
{
  pdiErrorTotal_ += n.pdi_error - p.pdi_error;
  epuErrorTotal_ += n.epu_error - p.epu_error;
  for (unsigned i = 0; i < 4; ++i)
  {
    EthercatPortDiagnostics & pt(portDiagnostics_[i]);
    pt.rxErrorTotal += n.port[i].rx_error - p.port[i].rx_error;
    pt.forwardedRxErrorTotal += n.forwarded_rx_error[i] - p.forwarded_rx_error[i];
    pt.lostLinkTotal += n.lost_link[i] - p.lost_link[i];
    pt.invalidFrameTotal += n.port[i].invalid_frame - p.port[i].invalid_frame;
  }
}

void EthercatDeviceDiagnostics::collect(EthercatCom *com, EtherCAT_SlaveHandler *sh)
{
  diagnosticsValid_ = false;
  diagnosticsFirst_ = false;

  // Check if device has been reset/power cycled using its node address
  // Node address initialize to 0 after device reset.
  // EML library will configure each node address to non-zero when it first starts
  // Device should respond to its node address, if it does not either:
  //  1. communication to device is not possible (lost/broken link)
  //  2. device was reset, and its fixed address setting is now 0
  {
    // Send a packet with both a Fixed address read (NPRW) and a positional read (APRD)
    // If the NPRD has a working counter == 0, but the APRD sees the correct number of devices,
    // then the node has likely been reset.
    // Also, get DL status register with nprd telegram

    // This is for the case of non-ethercat device where SlaveHandler is NULL
    if (sh == NULL)
    {
      diagnosticsValid_ = true;
      return;
    }

    EC_Logic *logic = sh->m_logic_instance;
    et1x00_dl_status dl_status;
    NPRD_Telegram nprd_telegram(logic->get_idx(),
                                sh->get_station_address(),
                                dl_status.BASE_ADDR,
                                logic->get_wkc(),
                                sizeof (dl_status),
                                (unsigned char*) &dl_status);
    // Use positional read to re-count number of devices on chain
    unsigned char buf[1];
    uint16_t address = 0x0000;
    APRD_Telegram aprd_telegram(logic->get_idx(), // Index
                                0, // Slave position on ethercat chain (auto increment address) (
                                address, // ESC physical memory address (start address)
                                logic->get_wkc(), // Working counter
                                sizeof (buf), // Data Length,
                                buf); // Buffer to put read result into



    // Chain both telegrams together
    nprd_telegram.attach(&aprd_telegram);

    EC_Ethernet_Frame frame(&nprd_telegram);

    // Send/Recv data from slave
    if (!com->txandrx_once(&frame))
    {
      // no response - broken link to device
      goto end;
    }

    devicesRespondingToNodeAddress_ = nprd_telegram.get_wkc();
    if (devicesRespondingToNodeAddress_ == 0)
    {
      // Device has not responded to its node address.
      if (aprd_telegram.get_adp() >= sh->m_router_instance->m_al_instance->get_num_slaves())
      {
        resetDetected_ = true;
        goto end;
      }
    }
    else if (devicesRespondingToNodeAddress_ > 1)
    {
      // Can't determine much if two (or more) devices are responding to same request.
      goto end;
    }
    else
    {
      resetDetected_ = false;
    }

    // fill in port status information
    for (unsigned i = 0; i < 4; ++i)
    {
      EthercatPortDiagnostics & pt(portDiagnostics_[i]);
      pt.hasLink = dl_status.hasLink(i);
      pt.isClosed = dl_status.isClosed(i);
      pt.hasCommunication = dl_status.hasCommunication(i);
    }
  }

  { // read and accumulate communication error counters
    et1x00_error_counters e;
    assert(sizeof (e) == (0x314 - 0x300));
    if (0 != EthercatDevice::readData(com, sh, e.BASE_ADDR, &e, sizeof (e), EthercatDevice::FIXED_ADDR))
    {
      goto end;
    }

    // If this previously tried to clear the error counters but d/n get a response
    // then use the newly read values to guess if they got cleared or not.
    if (errorCountersMayBeCleared_)
    {
      if (!e.isGreaterThan(errorCountersPrev_))
      {
        errorCountersPrev_.zero();
      }
      errorCountersMayBeCleared_ = false;
    }
    if (errorCountersPrev_.isGreaterThan(e))
    {
      ROS_ERROR("Device %d : previous port error counters less current values", sh->get_ring_position());
    }

    // Accumulate
    this->accumulate(e, errorCountersPrev_);
    errorCountersPrev_ = e;

    // re-read and clear communication error counters
    if (e.isGreaterThan(50))
    {
      if (0 != EthercatDevice::readWriteData(com, sh, e.BASE_ADDR, &e, sizeof (e), EthercatDevice::FIXED_ADDR))
      {
        // Packet got lost... Can't know for sure that error counters got cleared
        errorCountersMayBeCleared_ = true;
        goto end;
      }
      // We read and cleared error counters in same packet, accumulate any error counter differences
      this->accumulate(e, errorCountersPrev_);
      errorCountersPrev_.zero();
    }
  }

  // Everything was read successfully
  diagnosticsValid_ = true;

end:
  return;
}

void EthercatDeviceDiagnostics::publish(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned numPorts) const
{
  if (numPorts > 4)
  {
    assert(numPorts < 4);
    numPorts = 4;
  }
  assert(numPorts > 0);

  d.addf("Reset detected", "%s", (resetDetected_ ? "Yes" : "No"));
  d.addf("Valid", "%s", (diagnosticsValid_ ? "Yes" : "No"));

  d.addf("EPU Errors", "%lld", epuErrorTotal_);
  d.addf("PDI Errors", "%lld", pdiErrorTotal_);
  ostringstream os, port;
  for (unsigned i = 0; i < numPorts; ++i)
  {
    const EthercatPortDiagnostics & pt(portDiagnostics_[i]);
    port.str("");
    port << " Port " << i;
    os.str("");
    os << "Status" << port.str();
    d.addf(os.str(), "%s Link, %s, %s Comm",
           pt.hasLink ? "Has" : "No",
           pt.isClosed ? "Closed" : "Open",
           pt.hasCommunication ? "Has" : "No");
    os.str("");
    os << "RX Error" << port.str();
    d.addf(os.str(), "%lld", pt.rxErrorTotal);
    os.str("");
    os << "Forwarded RX Error" << port.str();
    d.addf(os.str(), "%lld", pt.forwardedRxErrorTotal);
    os.str("");
    os << "Invalid Frame" << port.str();
    d.addf(os.str(), "%lld", pt.invalidFrameTotal);
    os.str("");
    os << "Lost Link" << port.str();
    d.addf(os.str(), "%lld", pt.lostLinkTotal);
  }

  if (resetDetected_)
  {
    d.mergeSummaryf(d.ERROR, "Device reset likely");
  }
  else if (devicesRespondingToNodeAddress_ > 1)
  {
    d.mergeSummaryf(d.ERROR, "More than one device (%d) responded to node address", devicesRespondingToNodeAddress_);
  }
  else
  {
    if (diagnosticsFirst_)
    {
      d.mergeSummaryf(d.WARN, "Have not yet collected diagnostics");
    }
    else if (!diagnosticsValid_)
    {
      d.mergeSummaryf(d.WARN, "Could not collect diagnostics");
    }
    else
    {
      if (!portDiagnostics_[0].hasLink)
      {
        d.mergeSummaryf(d.WARN, "No link on port 0");
      }
    }
  }
}

void EthercatDevice::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  sh_ = sh;
  sh->set_fmmu_config(new EtherCAT_FMMU_Config(0));
  sh->set_pd_config(new EtherCAT_PD_Config(0));
}

EthercatDevice::EthercatDevice() : use_ros_(true)
{
  sh_ = NULL;
  command_size_ = 0;
  status_size_ = 0;
  newDiagnosticsIndex_ = 0;

  int error = pthread_mutex_init(&newDiagnosticsIndexLock_, NULL);
  if (error != 0)
  {
    ROS_FATAL("Initializing indexLock failed : %s", strerror(error));
    sleep(1); // wait for ros to flush rosconsole output
    exit(EXIT_FAILURE);
  }

  error = pthread_mutex_init(&diagnosticsLock_, NULL);
  if (error != 0)
  {
    ROS_FATAL("Initializing diagnositcsLock failed : %s", strerror(error));
    sleep(1); // wait for ros to flush rosconsole output
    exit(EXIT_FAILURE);
  }
}

void EthercatDevice::collectDiagnostics(EthercatCom *com)
{
  // Really, should not need this lock, since there should only be one thread updating diagnostics.
  pthread_mutex_lock(&diagnosticsLock_);

  // Get references to diagnostics... code is easier to read
  unsigned oldDiagnosticsIndex = (newDiagnosticsIndex_ + 1)&1;
  const EthercatDeviceDiagnostics &newDiag = deviceDiagnostics[newDiagnosticsIndex_];
  EthercatDeviceDiagnostics &oldDiag = deviceDiagnostics[oldDiagnosticsIndex];

  // copy new diagnostics values in old diagnostics, because diagnostic data use accumumlators
  oldDiag = newDiag;

  // Collect diagnostics data into "old" buffer.
  // This way the "new" buffer is never changed while the publishing thread may be using it.
  oldDiag.collect(com, sh_);

  // Got new diagnostics... swap buffers.
  // Publisher thread uses "new" buffer.  New to lock while swapping buffers.
  // Note : this is just changing an integer value, so it only takes a couple of instructions.
  pthread_mutex_lock(&newDiagnosticsIndexLock_);
  newDiagnosticsIndex_ = oldDiagnosticsIndex;
  pthread_mutex_unlock(&newDiagnosticsIndexLock_);

  // Done, unlock
  pthread_mutex_unlock(&diagnosticsLock_);
}

int EthercatDevice::readWriteData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address, void* buffer, uint16_t length, AddrMode addrMode)
{
  unsigned char *p = (unsigned char *) buffer;
  EC_Logic *logic = sh->m_logic_instance;

  NPRW_Telegram nprw_telegram(logic->get_idx(),
                              sh->get_station_address(),
                              address,
                              logic->get_wkc(),
                              length,
                              p);

  APRW_Telegram aprw_telegram(logic->get_idx(), // Index
                              -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address) (
                              address, // ESC physical memory address (start address)
                              logic->get_wkc(), // Working counter
                              length, // Data Length,
                              p); // Buffer to put read result into

  // Put read telegram in ros_ethercat_eml/ethernet frame
  EC_Telegram * telegram = NULL;
  if (addrMode == FIXED_ADDR)
  {
    telegram = &nprw_telegram;
  }
  else if (addrMode == POSITIONAL_ADDR)
  {
    telegram = &aprw_telegram;
  }
  else
  {
    assert(0);
    return -1;
  }

  // Put telegram in ros_ethercat_eml/ethernet frame
  EC_Ethernet_Frame frame(telegram);

  // Send/Recv data from slave
  if (!com->txandrx_once(&frame))
  {
    return -1;
  }

  // In some cases (clearing status mailbox) this is expected to occur
  if (telegram->get_wkc() != 3)
  {
    return -2;
  }

  return 0;
}

int EthercatDevice::readData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address, void* buffer, uint16_t length, AddrMode addrMode)
{
  unsigned char *p = (unsigned char *) buffer;
  EC_Logic *logic = sh->m_logic_instance;

  NPRD_Telegram nprd_telegram(logic->get_idx(),
                              sh->get_station_address(),
                              address,
                              logic->get_wkc(),
                              length,
                              p);

  APRD_Telegram aprd_telegram(logic->get_idx(), // Index
                              -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address) (
                              address, // ESC physical memory address (start address)
                              logic->get_wkc(), // Working counter
                              length, // Data Length,
                              p); // Buffer to put read result into

  // Put read telegram in ros_ethercat_eml/ethernet frame
  EC_Telegram * telegram = NULL;
  if (addrMode == FIXED_ADDR)
  {
    telegram = &nprd_telegram;
  }
  else if (addrMode == POSITIONAL_ADDR)
  {
    telegram = &aprd_telegram;
  }
  else
  {
    assert(0);
    return -1;
  }

  // Put telegram in ros_ethercat_eml/ethernet frame
  EC_Ethernet_Frame frame(telegram);

  // Send/Recv data from slave
  if (!com->txandrx(&frame))
  {
    return -1;
  }

  // In some cases (clearing status mailbox) this is expected to occur
  if (telegram->get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

int EthercatDevice::writeData(EthercatCom *com, EtherCAT_SlaveHandler *sh, uint16_t address, void const* buffer, uint16_t length, AddrMode addrMode)
{
  unsigned char const *p = (unsigned char const*) buffer;
  EC_Logic *logic = sh->m_logic_instance;

  NPWR_Telegram npwr_telegram(logic->get_idx(),
                              sh->get_station_address(),
                              address,
                              logic->get_wkc(),
                              length,
                              p);

  APWR_Telegram apwr_telegram(logic->get_idx(), // Index
                              -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address) (
                              address, // ESC physical memory address (start address)
                              logic->get_wkc(), // Working counter
                              length, // Data Length,
                              p); // Buffer to put read result into

  // Put read telegram in ros_ethercat_eml/ethernet frame
  EC_Telegram * telegram = NULL;
  if (addrMode == FIXED_ADDR)
  {
    telegram = &npwr_telegram;
  }
  else if (addrMode == POSITIONAL_ADDR)
  {
    telegram = &apwr_telegram;
  }
  else
  {
    assert(0);
    return -1;
  }

  // Put telegram in ros_ethercat_eml/ethernet frame
  EC_Ethernet_Frame frame(telegram);

  // Send/Recv data from slave
  if (!com->txandrx(&frame))
  {
    return -1;
  }

  if (telegram->get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

void EthercatDevice::ethercatDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned numPorts)
{
  if (numPorts > 4)
  {
    assert(numPorts < 4);
    numPorts = 4;
  }

  // By locking index, diagnostics double-buffer cannot be swapped.
  // Update thread is only allowed to change 'old' diagnostics buffer,
  //  so new buffer data cannot be changed while index lock is held.
  pthread_mutex_lock(&newDiagnosticsIndexLock_);

  const EthercatDeviceDiagnostics &newDiag = deviceDiagnostics[newDiagnosticsIndex_];
  newDiag.publish(d, numPorts);

  pthread_mutex_unlock(&newDiagnosticsIndexLock_);
}

void EthercatDevice::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  stringstream str;
  str << "EtherCAT Device (" << std::setw(2) << std::setfill('0') << sh_->get_ring_position() << ")";
  d.name = str.str();
  str.str("");
  str << sh_->get_product_code() << '-' << sh_->get_serial();
  d.hardware_id = str.str();

  d.message = "";
  d.level = 0;

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code", "%08x", sh_->get_product_code());
  d.addf("Serial", "%08x", sh_->get_serial());
  d.addf("Revision", "%08x", sh_->get_revision());

  this->ethercatDiagnostics(d, 4); //assume unknown device has 4 ports
}

void EthercatDevice::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer)
{
  // Clean up recycled status object before reusing it.
  diagnostic_status_.clearSummary();
  diagnostic_status_.clear();

  // If child-class does not implement multiDiagnostics(), fall back to using slave's diagnostic() function
  diagnostics(diagnostic_status_, buffer);
  vec.push_back(diagnostic_status_);
}
