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

#ifndef ETHERNET_INTERFACE_INFO_H
#define ETHERNET_INTERFACE_INFO_H

#include <string>
#include <stdint.h>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

struct EthtoolStats
{
  EthtoolStats();
  uint64_t rx_errors_;
  uint64_t rx_crc_errors_;
  uint64_t rx_frame_errors_;
  uint64_t rx_align_errors_;

  EthtoolStats& operator-=(const EthtoolStats& right);
};

struct InterfaceState
{
  InterfaceState() :
    up_(false), running_(false)
  {
  }
  bool up_;
  bool running_;
};

class EthernetInterfaceInfo
{
public:
  EthernetInterfaceInfo();
  void initialize(const std::string &interface);
  ~EthernetInterfaceInfo();

  /**
   * \brief Collect and append ethernet interface diagnostics
   *
   * \param d       Diagnostics status wrapper.
   */
  void publishDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

protected:
  //! Get ethtool stats from interface
  bool getEthtoolStats(EthtoolStats &stats);

  //! Get state (up,running) of interface
  bool getInterfaceState(InterfaceState &state);

  //! name of network interface (for example : eth0)
  std::string interface_;
  //! network socket for making ioctl requests
  int sock_;
  //! Number of stats available from ethtool ioctl
  unsigned n_stats_;
  //! buffer for NIC statistic values
  char *ethtool_stats_buf_;

  // Indexes of statistics that come from ethtool ioctl
  // An index of -1 indicates that statistic is not available from network driver
  int rx_error_index_;
  int rx_crc_error_index_;
  int rx_frame_error_index_;
  int rx_align_error_index_;

  //! Number of time master link went down
  unsigned lost_link_count_;

  //! Original statistics counts when initialize() was called.
  EthtoolStats orig_stats_;
  InterfaceState last_state_;
};

#endif //ETHERNET_INTERFACE_INFO_H
