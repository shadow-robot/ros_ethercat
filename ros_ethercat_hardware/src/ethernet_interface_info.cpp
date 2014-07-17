#include "ros_ethercat_hardware/ethernet_interface_info.h"
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <errno.h>

EthtoolStats::EthtoolStats() :
  rx_errors_(0),
  rx_crc_errors_(0),
  rx_frame_errors_(0),
  rx_align_errors_(0)
{
  //empty
}

EthtoolStats& EthtoolStats::operator-=(const EthtoolStats& right)
{
  this->rx_errors_ -= right.rx_errors_;
  this->rx_crc_errors_ -= right.rx_crc_errors_;
  this->rx_frame_errors_ -= right.rx_frame_errors_;
  this->rx_align_errors_ -= right.rx_align_errors_;
  return *this;
}

EthernetInterfaceInfo::EthernetInterfaceInfo() :
  sock_(-1),
  n_stats_(0),
  ethtool_stats_buf_(NULL),
  rx_error_index_(-1),
  rx_crc_error_index_(-1),
  rx_frame_error_index_(-1),
  rx_align_error_index_(-1)
{
}

EthernetInterfaceInfo::~EthernetInterfaceInfo()
{
  delete[] ethtool_stats_buf_;
  ethtool_stats_buf_ = NULL;
  if (sock_ >= 0)
    close(sock_);
}

void EthernetInterfaceInfo::initialize(const std::string &interface)
{
  interface_ = interface;

  // Need network socket to make interface requests ioctls
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0)
  {
    ROS_WARN("Cannot get control socket for ioctls : %s", strerror(errno));
    return;
  }

  // Get initial interface state
  getInterfaceState(last_state_);

  struct ifreq ifr;
  memset(&ifr, 0, sizeof (ifr));
  strncpy(ifr.ifr_name, interface_.c_str(), sizeof (ifr.ifr_name));

  // Determine number of statictics available from network interface
  struct ethtool_drvinfo drvinfo;
  drvinfo.cmd = ETHTOOL_GDRVINFO;
  ifr.ifr_data = (caddr_t) & drvinfo;
  int result = ioctl(sock_, SIOCETHTOOL, &ifr);
  if (result < 0)
  {
    ROS_WARN("Cannot get driver information for %s : %s", interface_.c_str(), strerror(errno));
    return;
  }
  n_stats_ = drvinfo.n_stats;
  if (n_stats_ < 1)
  {
    ROS_WARN("No NIC statistics available for %s", interface_.c_str());
    return;
  }

  unsigned strings_len = sizeof (ethtool_gstrings) + n_stats_ * ETH_GSTRING_LEN;
  char *strings_buf = new char[strings_len];
  memset(strings_buf, 0, strings_len);
  ethtool_gstrings* strings = (ethtool_gstrings*) strings_buf;

  strings->cmd = ETHTOOL_GSTRINGS;
  strings->string_set = ETH_SS_STATS;
  strings->len = n_stats_;
  ifr.ifr_data = (caddr_t) strings;
  result = ioctl(sock_, SIOCETHTOOL, &ifr);
  if (result < 0)
  {
    ROS_WARN("Cannot get statistics strings for %s : %s", interface_.c_str(), strerror(errno));
    delete[] strings_buf;
    return;
  }

  for (unsigned i = 0; i < n_stats_; ++i)
  {
    if (false)
    {
      char s[ETH_GSTRING_LEN + 1];
      strncpy(s, (const char*) &strings->data[i * ETH_GSTRING_LEN], ETH_GSTRING_LEN);
      s[ETH_GSTRING_LEN] = '\0';
      ROS_WARN("Stat %i : %s", i, s);
    }
    const char *stat_name = (const char*) &strings->data[i * ETH_GSTRING_LEN];
    if (strncmp("rx_errors", stat_name, ETH_GSTRING_LEN) == 0)
    {
      rx_error_index_ = i;
    }
    else if (strncmp("rx_crc_errors", stat_name, ETH_GSTRING_LEN) == 0)
    {
      rx_crc_error_index_ = i;
    }
    else if (strncmp("rx_frame_errors", stat_name, ETH_GSTRING_LEN) == 0)
    {
      rx_frame_error_index_ = i;
    }
    else if (strncmp("rx_align_errors", stat_name, ETH_GSTRING_LEN) == 0)
    {
      rx_align_error_index_ = i;
    }
  }

  // Everything is complete, allocate memory for ethtool_stats_ buffer
  // Since not all NICs provide ethtool statistics, use the presence of
  // ethtool_stats_ buffer to indicate initialization was a success.
  unsigned ethtool_stats_buf_len = sizeof (struct ethtool_stats) +n_stats_ * sizeof (uint64_t);
  ethtool_stats_buf_ = new char[ethtool_stats_buf_len];

  if (!getEthtoolStats(orig_stats_))
  {
    // Don't run if we can't get initial statitics
    ROS_WARN("Error collecting intial ethernet interface statistics");
    delete[] ethtool_stats_buf_;
    ethtool_stats_buf_ = NULL;
  }
}

bool EthernetInterfaceInfo::getInterfaceState(InterfaceState &state)
{
  struct ifreq ifr;
  memset(&ifr, 0, sizeof (ifr));
  strncpy(ifr.ifr_name, interface_.c_str(), sizeof (ifr.ifr_name));

  if (ioctl(sock_, SIOCGIFFLAGS, &ifr) < 0)
  {
    ROS_WARN("Cannot get interface flags for %s: %s", interface_.c_str(), strerror(errno));
    return false;
  }

  state.up_ = bool(ifr.ifr_flags & IFF_UP);
  state.running_ = bool(ifr.ifr_flags & IFF_RUNNING);
  return true;
}

bool EthernetInterfaceInfo::getEthtoolStats(EthtoolStats &s)
{
  if (!ethtool_stats_buf_)
    return false;

  struct ifreq ifr;
  memset(&ifr, 0, sizeof (ifr));
  strncpy(ifr.ifr_name, interface_.c_str(), sizeof (ifr.ifr_name));

  struct ethtool_stats *stats = (struct ethtool_stats *) ethtool_stats_buf_;
  stats->cmd = ETHTOOL_GSTATS;
  stats->n_stats = n_stats_;
  ifr.ifr_data = (caddr_t) stats;
  if (ioctl(sock_, SIOCETHTOOL, &ifr) < 0)
  {
    ROS_WARN("Cannot get NIC stats information for %s : %s", interface_.c_str(), strerror(errno));
    return false;
  }

  if (rx_error_index_ >= 0)
  {
    s.rx_errors_ = stats->data[rx_error_index_];
  }
  if (rx_crc_error_index_ >= 0)
  {
    s.rx_crc_errors_ = stats->data[rx_crc_error_index_];
  }
  if (rx_frame_error_index_ >= 0)
  {
    s.rx_frame_errors_ = stats->data[rx_frame_error_index_];
  }
  if (rx_align_error_index_ >= 0)
  {
    s.rx_align_errors_ = stats->data[rx_align_error_index_];
  }

  return true;
}

void EthernetInterfaceInfo::publishDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  d.add("Interface", interface_);

  // TODO : collect and publish information on whether interface is up/running
  InterfaceState state;
  if (getInterfaceState(state))
  {
    if (!state.running_ && last_state_.running_)
    {
      ++lost_link_count_;
    }

    if (state.up_ && !state.running_)
    {
      d.mergeSummary(d.ERROR, "No link");
    }
    else if (!state.up_)
    {
      d.mergeSummary(d.ERROR, "Interface down");
    }

    d.addf("Interface State", "%s UP, %s RUNNING", state.up_ ? "" : "NOT",
           state.running_ ? "" : "NOT");
    last_state_ = state;
  }
  else
  {
    d.add("Iface State", "ERROR");
  }
  d.add("Lost Links", lost_link_count_);

  EthtoolStats stats;
  bool have_stats = getEthtoolStats(stats);
  stats -= orig_stats_; //subtract off orignal counter values

  if (have_stats && (rx_error_index_ >= 0))
    d.addf("RX Errors", "%llu", stats.rx_errors_);
  else
    d.add("RX Errors", "N/A");

  if (have_stats && (rx_crc_error_index_ >= 0))
    d.addf("RX CRC Errors", "%llu", stats.rx_crc_errors_);
  else
    d.add("RX CRC Errors", "N/A");

  if (have_stats && (rx_frame_error_index_ >= 0))
    d.addf("RX Frame Errors", "%llu", stats.rx_frame_errors_);
  else
    d.add("RX Frame Errors", "N/A");

  if (have_stats && (rx_align_error_index_ >= 0))
    d.addf("RX Align Errors", "%llu", stats.rx_align_errors_);
  else
    d.add("RX Align Errors", "N/A");

}
