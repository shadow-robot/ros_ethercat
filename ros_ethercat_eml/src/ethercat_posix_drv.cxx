// $Id:$
//===========================================================================
//	This file is part of "EtherCAT Master Library".
//	Copyright (C) 2005 FMTC vzw, Diamant Building, A. Reyerslaan 80,
//	B-1030 Brussels, Belgium.
//
//	EtherCAT Master Library is free software; you can redistribute it
//	and/or modify it under the terms of the GNU General Public License
//	as published by the Free Software Foundation; either version 2 or
//	(at your option) any later version.
//
//	EtherCAT Master Code is distributed in the hope that it will be
//	useful, but WITHOUT ANY WARRANTY; without even the implied
//	warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//	PURPOSE. See the GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with the EtherCAT Master Library; if not, write to the Free
//	Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
//	02111-1307 USA.
//
//	EtherCAT, the EtherCAT trade name and logo are the intellectual
//	property of, and protected by Beckhoff. You can use "EtherCAT
//	Master Library" for creating and/or selling or otherwise
//	distributing an EtherCAT network master under the terms of the
//	EtherCAT Master License.
//
//	You should have received a copy of the EtherCAT Master License
//	along with the EtherCAT Master Library; if not, write to Beckhoff
//	Automation GmbH, Eiserstrasse 5, D-33415 Verl, Germany.
//===========================================================================

/*
 Contributed by RvdM, October, 5th, 2007.
 Purpose to use EML over PREEMPT_RT kernel
 Based on rtnet/xenomai driver
 */

#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <assert.h>
#include <string.h>

#include <ros_ethercat_eml/netif.h>
#include <ros_ethercat_eml/ethercat_log.h>

#include <time.h>
#include <unistd.h>

// Maximum tries to send and receive a message
#define MAX_TRIES_TX 10
// Timeout for receiving and transmitting messages is 2000 us
// specifying 1000us or lower did not work on plain Linux.
// recv() returned immediately with EWOULDBLOCK.
// May/must be different for xgPREEMPT_RT kernels.
#define TIMEOUT_USEC 20000

// Socket timeout - so input_thread can exit
#define SOCKET_TIMEOUT_USEC 10000

// Maximum times the master may retry to create or close a socket
#define MAX_TRIES_SOCKET 10
// Ethernet Header is 14 bytes: 2 * MAC ADDRESS + 2 bytes for the type
#define HEADER_SIZE 14
#define MAX_ETH_DATA 1500

#define NSEC_PER_SEC 1000000000

// Size of buffer that holds message from strerror
#define ERRBUF_LEN 60

static char* my_strerror(int errnum, char *buf, size_t buflen)
{
  assert(buflen > 0);
  assert(buf != NULL);
  buf[0] = '\0';
  if (strerror_r(errnum, buf, buflen) != 0)
  {
    snprintf(buf, buflen, "N%d", errnum);
  }
  return buf;
}

static void my_usleep(uint32_t usec)
{
  char errbuf[ERRBUF_LEN];
  assert(usec < 1000000);
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = usec * 1000;
  while (nanosleep(&req, &rem) != 0)
  {
    int error = errno;
    ec_log(EC_LOG_ERROR, "%s : Error : %s\n", __func__, my_strerror(error, errbuf, sizeof (errbuf)));
    if (error != EINTR)
    {
      break;
    }
    req = rem;
  }
  return;
}

// No longer set socket timeout, but insteads set timeout variable used by pthread_cond_timedwait.
// Produces the simular semantics as setting socket timeout in non-threaded posix driver
// Timeout value it on microseconds

int set_socket_timeout(struct netif* ni, int64_t timeout)
{
  if ((timeout * 1000) >= NSEC_PER_SEC)
  {
    ec_log(EC_LOG_FATAL, "%s: timeout is too large : %ld\n", __func__, timeout);
    assert(timeout * 1000 < NSEC_PER_SEC);
    return -1;
  }
  ni->timeout_us = timeout;
  return 0;
}

static int init_socket(const char* interface)
{
  int sock;
  int tries = 0;
  struct sockaddr_ll addr;
  struct ifreq ifr;

  char errbuf[ERRBUF_LEN];

  while (((sock = socket(PF_PACKET, SOCK_RAW, htons(0x88A4))) < 0) && tries < MAX_TRIES_SOCKET)
  {
    int error = errno;
    fprintf(stderr, "Couldn't open raw socket for interface %s : %s\n",
            interface,
            my_strerror(error, errbuf, sizeof (errbuf)));
    sleep(1);
    tries++;
  }

  if (sock < 0)
  {
    perror("Failed to create socket");
    return -1;
  }

  //printf("Socket created: socket id: %d\n", sock);

  int rv, index_ioctl;
  strncpy(ifr.ifr_name, interface, IFNAMSIZ);
  if ((rv = ioctl(sock, SIOCGIFFLAGS, &ifr)) < 0)
  {
    perror("Cannot get interface flags");
    close(sock);
    return -1;
  }
  if (!(ifr.ifr_flags & IFF_UP))
  {
    fprintf(stderr,
            "Interface %s is not UP\n"
            " try : ifup %s\n"
            ,
            interface, interface);
    return -1;
  }

  if ((index_ioctl = ioctl(sock, SIOCGIFINDEX, &ifr)) < 0)
  {
    perror("Cannot get interface index");
    close(sock);
    return -1;
  }
  //printf("Got interface: index: %d for %s\n", index_ioctl, ifr.ifr_name);

  struct timeval tv;
  tv.tv_sec = (SOCKET_TIMEOUT_USEC) / 1000000;
  tv.tv_usec = (SOCKET_TIMEOUT_USEC) % 1000000;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void*) &tv, sizeof (tv)) != 0)
  {
    perror("Aborting: Cannot set timeout");
    return -1;
  }
  else
  {
    //printf("Input thread recv timeout has been set to %u usecs.\n", SOCKET_TIMEOUT_USEC);
  }

  memset(&addr, 0, sizeof (addr));
  addr.sll_family = AF_PACKET;
  addr.sll_protocol = htons(0x88A4);
  addr.sll_ifindex = ifr.ifr_ifindex;

  if ((bind(sock, (struct sockaddr *) &addr, sizeof (addr))) < 0)
  {
    perror("Cannot bind to local ip/port");
    close(sock);
    return -1;
  }

  return sock;
}

int close_socket(struct netif *ni)
{
  assert(ni != NULL);
  char errbuf[ERRBUF_LEN];

  // If sock is < 0, things have not been started
  if (ni->socket_private < 0)
  {
    ec_log(EC_LOG_ERROR, "%s: close called twice on netif\n", __func__);
    return -1;
  }

  // First try stopping input thread
  ec_log(EC_LOG_INFO, "%s : Stopping input thread...\n", __func__);
  ni->stop = true;
  int tries = 0;
  for (tries = 0; tries < 10; ++tries)
  {
    my_usleep(SOCKET_TIMEOUT_USEC);
    if (ni->is_stopped)
    {
      ec_log(EC_LOG_INFO, "%s : Input thread has stopped\n", __func__);
      break;
    }
  }

  // Force input thread to exit
  if (!ni->is_stopped)
  {
    ec_log(EC_LOG_ERROR, "%s: input thread d/n stop, cancelling thread\n", __func__);
    int error = pthread_cancel(ni->input_thread);
    if (error != 0)
    {
      ec_log(EC_LOG_ERROR, "%s: error cancelling input thread : %s\n", __func__,
             my_strerror(error, errbuf, sizeof (errbuf)));
      return -1;
    }
    my_usleep(SOCKET_TIMEOUT_USEC);
  }

  int ret = close(ni->socket_private);
  tries = 1;
  while (ret < 0 && tries < MAX_TRIES_SOCKET)
  {
    ret = close(ni->socket_private);
    tries++;
    sleep(1);
  }
  if (ret < 0)
    perror("Failed to close socket");

  ni->socket_private = -1;

  if (ni->is_stopped)
  {
    free(ni);
  }

  return ret;
}

struct eth_msg
{
  uint8_t ether_dhost[ETH_ALEN]; /* destination eth addr */
  uint8_t ether_shost[ETH_ALEN]; /* source ether addr    */
  uint8_t ether_type[2]; /* packet type ID field */
  unsigned char data[MAX_ETH_DATA];
};

// Send packet, if successfull add handle to list of packet awaiting a response.
// Returns postive handle to packet list for success.
// Returns negative value for errors
// Assumes txandrx_mut is held when this function is called

static int low_level_output(struct EtherCAT_Frame * frame, struct netif * netif)
{
  struct netif *ni = netif;
  assert(ni != NULL);
  assert(pthread_mutex_lock(&ni->txandrx_mut) == EDEADLK);

  char errbuf[ERRBUF_LEN];

  // Is there a slot to send packet with?
  if (ni->unclaimed_packets >= MAX_UNCLAIMED_PACKETS)
  {
    // Maybe put a condition variable here is the future.
    ec_log(EC_LOG_FATAL, "%s: too many outstanding packets : %d\n",
           __func__,
           ni->unclaimed_packets);
    ++ni->counters.tx_full;
    return -1;
  }

  // Find empty spot in list to store information about the sent packet
  struct outstanding_pkt* pkt = NULL;
  unsigned pkt_index;
  {
    unsigned xx;
    for (xx = 0; xx < PKT_LIST_SIZE; ++xx)
    {
      pkt_index = (ni->next_pkt_index + xx) % PKT_LIST_SIZE;
      struct outstanding_pkt* i_pkt = &ni->pkt_list[pkt_index];
      if (i_pkt->is_free)
      {
        pkt = i_pkt;
        break;
      }
    }
  }

  // Couldn't find empty slot
  if (pkt == NULL)
  {
    ec_log(EC_LOG_FATAL, "%s: outstanding packet list is full\n", __func__);
    ++ni->counters.tx_full;
    return -1;
  }

  // higher level protocol error. Attempt to map too much data in one ethernet frame
  struct eth_msg msg_to_send;
  memset(msg_to_send.data, 0x00, MAX_ETH_DATA);
  int len_dump = framedump(frame, msg_to_send.data, MAX_ETH_DATA);
  if (len_dump == 0)
  {
    ec_log(EC_LOG_FATAL, "%s: message buffer overflow\n", __func__);
    ++ni->counters.tx_error;
    return -1;
  }

  // Send packet
  int msg_len = len_dump + ETH_ALEN + ETH_ALEN + 2;
  int sock = ni->socket_private;
  // Destination address is broadcast MAC address
  // FIXME Is this also valid for EtherCAT UDP ethernet frames?
  memset(msg_to_send.ether_dhost, 0xFF, ETH_ALEN);
  // Source address, last 3 bytes are used as markers
  memcpy(msg_to_send.ether_shost, ni->hwaddr, ETH_ALEN);
  assert(pkt_index <= 0xFF);
  msg_to_send.ether_shost[3] = pkt_index;
  ni->tx_seqnum = (ni->tx_seqnum + 1) & 0xFFFF;
  msg_to_send.ether_shost[4] = ((ni->tx_seqnum) >> 8) & 0xFF;
  msg_to_send.ether_shost[5] = ((ni->tx_seqnum) >> 0) & 0xFF;

  // EtherType is ethercat
  msg_to_send.ether_type[0] = 0x88;
  msg_to_send.ether_type[1] = 0xA4;

  // Pad the packet so the data (src + dst + ethertype + payload) is at least 60 bytes
  // This prevents wireshark from complianing about parsing errors
  int pkt_len = (msg_len < 60) ? 60 : msg_len;

  // Get send time
  if (clock_gettime(CLOCK_REALTIME, &pkt->tx_time) != 0)
  {
    int error = errno;
    ec_log(EC_LOG_FATAL, "%s: Could not get send_time : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
    ++ni->counters.tx_error;
    return -1;
  }

  // Send packet - sending should not block.
  int len_send = send(sock, (unsigned char *) &msg_to_send, pkt_len, MSG_DONTWAIT);
  if (len_send < 0)
  {
    int error = errno;
    if (error == ENETDOWN)
    {
      ++ni->counters.tx_net_down;
      if ((ni->counters.tx_net_down & 0xFFF) == 1)
      {
        ec_log(EC_LOG_FATAL, "%s: %llu times : %s\n",
               __func__,
               (unsigned long long) ni->counters.tx_net_down,
               my_strerror(error, errbuf, sizeof (errbuf)));
      }
    }
    else if ((error == EWOULDBLOCK) || (error == EAGAIN))
    {
      // On some systems EAGAIN != EWOULDBLOCK and either error code can be returned
      ++ni->counters.tx_would_block;
      if ((ni->counters.tx_would_block & 0xFFF) == 1)
      {
        ec_log(EC_LOG_FATAL, "%s: %llu times : Cannot Send : would block\n",
               __func__,
               (unsigned long long) ni->counters.tx_would_block);
      }
    }
    else if (error == ENOBUFS)
    {
      ++ni->counters.tx_no_bufs;
      if ((ni->counters.tx_no_bufs & 0xFFF) == 1)
      {
        ec_log(EC_LOG_FATAL, "%s: %llu times : Cannot Send : %s\n",
               __func__,
               (unsigned long long) ni->counters.tx_no_bufs,
               my_strerror(error, errbuf, sizeof (errbuf)));
      }
    }
    else
    {
      static int last_error = 0;
      ++ni->counters.tx_error;
      if ((error != last_error) || ((ni->counters.tx_error & 0xFFF) == 1))
      {
        ec_log(EC_LOG_FATAL, "%s: %llu times : Cannot Send : %s\n",
               __func__,
               (unsigned long long) ni->counters.tx_error,
               my_strerror(error, errbuf, sizeof (errbuf)));
      }
      last_error = error;
    }
    return -1;
  }
  if (len_send != pkt_len)
  {
    ec_log(EC_LOG_FATAL, "%s: Incomplete send, sent %d or %d bytes\n",
           __func__,
           len_send, pkt_len);
    ++ni->counters.tx_error;
    return -1;
  }

  // Put packet in list of outstanding packets
  ++ni->unclaimed_packets;
  pkt->is_free = false;
  pkt->frame = frame;
  memcpy(pkt->ether_shost, msg_to_send.ether_shost, ETH_ALEN);

  // Generate handle for sent packet
  int handle = 0xFFFFFF &
    ((pkt->ether_shost[3] << 16) |
     (pkt->ether_shost[4] << 8) |
     (pkt->ether_shost[5] << 0));

  // Start searching when picking next open packet slot
  ni->next_pkt_index = (ni->next_pkt_index + 1) % PKT_LIST_SIZE;

  ++ni->counters.sent;
  assert(handle >= 0);
  return handle;
}

//  Last two bytes of src MAC address lets us determine seqnum of packet

static unsigned parse_seqnum(uint8_t ether_shost[MAC_ADDRESS_SIZE])
{
  unsigned seqnum =
    (((unsigned) ether_shost[4]) << 8) |
    (((unsigned) ether_shost[5]) << 0);
  return seqnum;
}

enum input_retcode
{
  RECOVERABLE_ERROR = 0, UNRECOVERABLE_ERROR = -1, SUCCESS = 1
};

// Receives packet and stores it in pkt_list
// Returns 1 for success
// Returns -1 for unrecoverable errors (socket problem, not enough input buffers)
// Returns 0 for a timeout (no packet ready), or recoverable error (invalid packet received, bad packet format)
// Assumes txandrx_mut is held when this function is called

static enum input_retcode low_level_input(struct netif * ni)
{
  assert(ni != NULL);
  assert(pthread_mutex_lock(&ni->txandrx_mut) == EDEADLK);

  char errbuf[ERRBUF_LEN];

  // Find empty packet buffer to hold recieved packet
  struct pkt_buf *buf = NULL;
  {
    unsigned buf_index;
    for (buf_index = 0; buf_index < BUF_LIST_SIZE; ++buf_index)
    {
      struct pkt_buf *i_buf = &ni->buf_list[buf_index];
      if (i_buf->is_free)
      {
        buf = i_buf;
        break;
      }
    }
  }

  if (buf == NULL)
  {
    // No buffer to hold incoming packet
    ec_log(EC_LOG_FATAL, "%s : EtherCAT fatal: packet buffer list if full\n", __func__);
    return UNRECOVERABLE_ERROR;
  }
  assert(sizeof (struct eth_msg) < sizeof (buf->data)); //Const assert?
  struct eth_msg *msg_received = (struct eth_msg *) buf->data;

  // Try to receive message from socket
  // Don't hold mutex while blocking on recv, also mark buffer as
  // non-free, so it is not used by a different rx call.
  int sock = ni->socket_private;
  buf->is_free = false;
  pthread_mutex_unlock(&ni->txandrx_mut);
  int len_recv = recv(sock, msg_received, sizeof (*msg_received), 0);
  pthread_mutex_lock(&ni->txandrx_mut);
  buf->is_free = true;

  if (len_recv < 0)
  {
    int error = errno;
    if (error != EAGAIN)
    { // log error, for anthing other than a timeout
      ec_log(EC_LOG_ERROR, "%s: Cannot receive msg: %s\n",
             __func__,
             my_strerror(error, errbuf, sizeof (errbuf)));
      if (error == EINTR)
      {
        return RECOVERABLE_ERROR;
      }
      else
      {
        return UNRECOVERABLE_ERROR;
      }
    }
    return RECOVERABLE_ERROR;
  }

  if (len_recv <= sizeof (ETH_ALEN + ETH_ALEN + 2))
  {
    ec_log(EC_LOG_ERROR, "%s: recieved runt packet: %d\n", __func__, len_recv);
    ++ni->counters.rx_runt_pkt;
    return RECOVERABLE_ERROR; // we did receive a packet
  }

  if (((msg_received->ether_type[0]) != 0x88) || (msg_received->ether_type[1]) != 0xA4)
  {
    ec_log(EC_LOG_ERROR, "%s: No EtherCAT msg!\n", __func__);
    ++ni->counters.rx_not_ecat;
    return RECOVERABLE_ERROR; // we did receive a packet
  }

  // Use 2nd and 3rd byte of source max address to identify instance of EML that sent this out
  // Ignore packet with HW addresses from other EML libraries
  if ((msg_received->ether_shost[1] != ni->hwaddr[1]) ||
      (msg_received->ether_shost[2] != ni->hwaddr[2]))
  {
    ++ni->counters.rx_other_eml;
    if ((ni->counters.rx_other_eml & 0x3FF) == 1)
    {
      ec_log(EC_LOG_WARNING,
             "%s: received %llu packets sent out from another EML instance\n",
             __func__, (unsigned long long) ni->counters.rx_other_eml);
    }
    return RECOVERABLE_ERROR;
  }

  // 3rd byte of source MAC address tells us where outstanding packet should be in queue
  unsigned pkt_index = 0xFF & msg_received->ether_shost[3];
  if (pkt_index >= PKT_LIST_SIZE)
  {
    ec_log(EC_LOG_ERROR, "%s: packet doesn't belong in queue, bad index %d", __func__, pkt_index);
    ++ni->counters.rx_bad_index;
    return RECOVERABLE_ERROR;
  }
  struct outstanding_pkt *pkt = &ni->pkt_list[pkt_index];

  unsigned rx_seqnum = parse_seqnum(msg_received->ether_shost);

  // Print warning if this packet is a repeat or is recieved out-of-order
  //  This shouldn't happen with normal setup... It may be worthwhile
  //  know about this odd situation if it occurs.
  int16_t diff = (rx_seqnum - ni->rx_seqnum) & 0xFFFF;
  // diff == 1 : Good: this is the next expected packet..
  // diff == 0 : Bad: seems like a duplicate packet has been recieved,
  // diff <  1 : Bad: seems like an old got packets out-of-order, or old packet has been re-received
  // diff >  1 : Ok: could mean previous <diff> packets have been
  //             lost, and this is first to make it back successfully
  if (diff == 0)
  {
    ec_log(EC_LOG_ERROR, "low_level_input: error, got packet with duplicate seqnum %d\n",
           rx_seqnum);
    ++ni->counters.rx_dup_seqnum;
    // allow following checks to drop packet
  }
  else if (diff < 0)
  {
    // Newer packet has already been recieved, somehow packets got
    // reordered.  This should not happen on a raw ethernet chain.
    // Could occur if running ethercat over IP/UDP.
    // Also, could be caused by buffering in OS.
    ec_log(EC_LOG_ERROR,
           "low_level_input: warning : got packet in incorrect order: got %d, expected %d\n",
           rx_seqnum, (ni->rx_seqnum + 1) & 0xFFFF);
    ++ni->counters.rx_bad_order;
    // Don't drop packet might still be uncollected --
    // Allow following to filter it out.
  }

  // Determine if received seqnum matches seqnum used when sending packet
  unsigned tx_seqnum = parse_seqnum(pkt->ether_shost);
  if (rx_seqnum != tx_seqnum)
  {
    ec_log(EC_LOG_ERROR,
           "low_level_input: got packet with invalid seqnum: got %d, expected %d\n"
           "    next packet will be sent with seqnum %d\n"
           "    last received packet had seqnum %d\n"
           ,
           rx_seqnum, tx_seqnum, ni->tx_seqnum, ni->rx_seqnum);
    ++ni->counters.rx_bad_seqnum;
    return RECOVERABLE_ERROR;
  }

  // Make sure packet has not arrived already
  if ((pkt->buf != NULL))
  {
    ec_log(EC_LOG_ERROR, "low_level_input: got duplicate packet?\n");
    ++ni->counters.rx_dup_pkt;
    return RECOVERABLE_ERROR;
  }

  // Record most recently received seqnum
  ni->rx_seqnum = rx_seqnum;

  // Make sure packet has not been collected with rx
  if ((pkt->is_free == true))
  {
    ec_log(EC_LOG_ERROR,
           "%s: got packet that has already been collected with rx(), increase socket timeout?\n",
           __func__);
    ++ni->counters.rx_late_pkt;
    // Keep track of how long it actually took for the late packet to get back.
    struct timespec rx_time;
    if (clock_gettime(CLOCK_REALTIME, &rx_time) != 0)
    {
      int error = errno;
      ec_log(EC_LOG_FATAL, "%s: Could not get recv time for late packet : %s\n",
             __func__,
             my_strerror(error, errbuf, sizeof (errbuf)));
    }
    else
    {
      unsigned round_trip_time_us =
        (rx_time.tv_sec - pkt->tx_time.tv_sec) * 1000000 +
        (rx_time.tv_nsec - pkt->tx_time.tv_nsec) / 1000;
      ni->counters.rx_late_pkt_rtt_us = round_trip_time_us;
      ni->counters.rx_late_pkt_rtt_us_sum += round_trip_time_us;
      ec_log(EC_LOG_ERROR,
             "%s: late packet arrived in %u us, timeout set to %u us\n",
             __func__, round_trip_time_us, ni->timeout_us);
    }
    return RECOVERABLE_ERROR;
  }

  // Packet passes all checks... mark outstanding packet as having
  // received reply - mark buffer as being used.
  buf->is_free = false;
  pkt->buf = buf;

  // Signal anybody waiting on sent packet
  int error = pthread_cond_broadcast(&pkt->rx_cond);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s: cond broadcast : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
    return UNRECOVERABLE_ERROR;
  }

  ++ni->counters.received;
  return SUCCESS;
}

// Thread that continuously polls input and puts it into queue

void* low_level_input_thread_func(void* data)
{
  char errbuf[ERRBUF_LEN];

  // Increase priority of input thread
  struct sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  int error = pthread_setschedparam(pthread_self(), policy, &thread_param);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s : Setting thread sched : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
  }

  struct netif * ni = (struct netif *) data;
  ec_log(EC_LOG_INFO, "INFO: Starting input thread\n");
  bool stop = false;
  while (stop == false)
  {
    error = pthread_mutex_lock(&ni->txandrx_mut);
    assert(error == 0);
    stop = ni->stop;
    if (low_level_input(ni) == UNRECOVERABLE_ERROR)
    {
      ec_log(EC_LOG_FATAL, "%s : Unrecoverable error on input thread\n", __func__);
      // Some unrecoverable error can be recovered with external conditions (ENETDOWN)
      // However, don't spin on recv if it will immediately return an error code
      //stop = true;
      my_usleep(10000);
    }
    error = pthread_mutex_unlock(&ni->txandrx_mut);
    assert(error == 0);
  }
  ec_log(EC_LOG_INFO, "INFO: Input thread is exiting\n");
  ni->is_stopped = true;
  return NULL;
}

static void init_buf(struct pkt_buf *buf)
{
  buf->is_free = true;
}

static bool init_pkt(struct outstanding_pkt *pkt)
{
  pkt->is_free = true;
  pkt->buf = NULL;
  pkt->frame = NULL;
  memset(pkt->ether_shost, 0, sizeof (pkt->ether_shost));

  int error;
  char errbuf[ERRBUF_LEN];

  error = pthread_cond_init(&pkt->rx_cond, NULL);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s : Initializing rx condition var failed : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
    return false;
  }

  return true;
}

// Tries lookup of packet from queue using handle
// returns NULL if handle is stall/invalid, or pointer to outstanding pkt otherwise;
// Assumes txandrx mutex is already held

static struct outstanding_pkt *low_level_lookup(struct EtherCAT_Frame * frame, struct netif * ni,
                                                int handle)
{
  assert(frame != NULL);
  assert(ni != NULL);
  assert(pthread_mutex_lock(&ni->txandrx_mut) == EDEADLK);

  // Make sure handle makes sense, also make sure it is not a negative number
  if (((handle >> 24) & 0xFF) != 0)
  {
    ec_log(EC_LOG_ERROR, "low_level_lookup: called with invalid handle 0x%08X\n", handle);
    return NULL;
  }

  // Source MAC address packet should have
  // First 3 bytes are sent by netif source MAC address
  // Second 3 byte are encoded in handle.
  u_int8_t ether_shost[ETH_ALEN];
  memcpy(ether_shost, ni->hwaddr, ETH_ALEN);
  ether_shost[3] = (handle >> 16) & 0xFF;
  ether_shost[4] = (handle >> 8) & 0xFF;
  ether_shost[5] = (handle >> 0) & 0xFF;

  // 3th byte of source MAC address tells us where outstanding packet should be in queue
  unsigned pkt_index = 0xFF & ether_shost[3];
  if (pkt_index >= PKT_LIST_SIZE)
  {
    ec_log(EC_LOG_ERROR, "low_level_lookup: handle 0x%08X references bad pkt_index 0x%08X\n",
           handle, pkt_index);
    return NULL; //error
  }
  struct outstanding_pkt *pkt = &ni->pkt_list[pkt_index];

  // Check frame stored for outstanding packet against frame that was passed in
  if (pkt->frame != frame)
  {
    ec_log(EC_LOG_ERROR, "low_level_lookup: handle frame d/n match passed in frame \n");
    return NULL; //error
  }

  // Check MAC address encoded in handle against mac address packet was sent out with
  if (memcmp(pkt->ether_shost, ether_shost, ETH_ALEN) != 0)
  {
    ec_log(EC_LOG_ERROR, "low_level_lookup: tried collecting with stale handle \n");
    return NULL; //error
  }

  // Make sure packet was not already collected
  if (pkt->is_free != false)
  {
    ec_log(EC_LOG_ERROR, "low_level_lookup: tried collecting with same handle twice \n");
    return NULL; //error
  }

  return pkt;
}

// Drop packet from queue of outstanding packets.
// Returns false if packet d/n seem to exist
// Assumes txandrx mutex is already held

bool low_level_release(struct EtherCAT_Frame * frame, struct netif * ni, int handle)
{
  assert(pthread_mutex_lock(&ni->txandrx_mut) == EDEADLK);

  // Lookup pointer to packet using handle
  struct outstanding_pkt *pkt = low_level_lookup(frame, ni, handle);
  if (pkt == NULL)
  {
    return false;
  }

  // Mark packet buffer as availible
  if (pkt->buf != NULL)
  {
    init_buf(pkt->buf);
  }

  // Mark outstanding pkt as unused
  pkt->is_free = true;
  pkt->buf = NULL;
  pkt->frame = NULL;
  // Don't clear source MAC address, to allow detection of packets
  // that are received late, (rx was called before packet got back)

  // One less outstanding packet
  assert(ni->unclaimed_packets > 0);
  --ni->unclaimed_packets;

  // Packets are only released when they are not recieved
  ++ni->counters.dropped;

  return true;
}

enum dequeue_retcode
{
  DEQUEUE_SUCCESS = 1, DEQUEUE_ERROR = -1, DEQUEUE_NOT_FOUND = 0
};

// Tries retrieving packet from queue using handle
// returns negative values for error, 0 if no packet is in queue, or postive value if packet is found
// Assumes txandrx mutex is already held

static int low_level_dequeue(struct EtherCAT_Frame * frame, struct netif * ni, int handle)
{
  assert(pthread_mutex_lock(&ni->txandrx_mut) == EDEADLK);

  // Lookup pointer to packet using handle
  struct outstanding_pkt *pkt = low_level_lookup(frame, ni, handle);
  if (pkt == NULL)
  {
    return DEQUEUE_ERROR;
  }

  // OK, handle is good...
  // has response packet been received ?
  if (pkt->buf == NULL)
  {
    return DEQUEUE_NOT_FOUND; // packet has not been recieved
  }
  assert(pkt->buf->is_free == false);

  // Response packet has been received...

  // Get pointer to ethernet packet payout
  assert(sizeof (struct eth_msg) < sizeof (pkt->buf->data)); //Const assert?
  struct eth_msg *msg_received = (struct eth_msg *) pkt->buf->data;

  // Mark packet buffer as availible
  init_buf(pkt->buf);

  // Mare outstanding pkt as unused
  init_pkt(pkt);

  int success = framebuild(frame, msg_received->data);
  if (success != 0)
  {
    // FIXME decent error handling here
    ec_log(EC_LOG_ERROR, "low_level_input: framebuilding failed!\n");
    return DEQUEUE_ERROR;
  }

  // One less unclaimed packet
  assert(ni->unclaimed_packets > 0);
  --ni->unclaimed_packets;

  ++ni->counters.collected;
  return DEQUEUE_SUCCESS; // Good
}

// Transmits a packet.
// Returns a negative value for errors, or positive integer handle
//  handle and frame* should be used to get response with ec_posix_rx()

static int ec_posix_tx(struct EtherCAT_Frame * frame, struct netif * ni)
{
  assert(ni != NULL);

  int error = pthread_mutex_lock(&ni->txandrx_mut);
  assert(error == 0);
  int handle = low_level_output(frame, ni);
  error = pthread_mutex_unlock(&ni->txandrx_mut);
  assert(error == 0);
  return handle;
}

// Receives a packet that has been sent with ec_posix_tx()
// Returns true for success, false for errors or timeout.

static bool ec_posix_rx_common(struct EtherCAT_Frame * frame, struct netif * ni, int handle,
                               bool mayblock)
{
  assert(ni != NULL);
  assert(frame != NULL);
  char errbuf[ERRBUF_LEN];

  // Find rx mutex for outstanding packet
  struct outstanding_pkt *pkt;
  int error = pthread_mutex_lock(&ni->txandrx_mut);
  assert(error == 0);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s: error locking mutex : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
    return false;
  }
  pkt = low_level_lookup(frame, ni, handle);

  // Bad handle, double rx, etc...
  if (pkt == NULL)
  {
    pthread_mutex_unlock(&ni->txandrx_mut);
    return false;
  }

  // Try pulling result from queue
  int result = low_level_dequeue(frame, ni, handle);

  // If this can block, try waiting for condition signal
  if (mayblock)
  {
    // Wait until <timeout_us> after packet was sent
    struct timespec timeout = pkt->tx_time;
    timeout.tv_nsec += (ni->timeout_us * 1000);
    if (timeout.tv_nsec >= NSEC_PER_SEC)
    {
      timeout.tv_nsec -= NSEC_PER_SEC;
      ++timeout.tv_sec;
    }
    assert(timeout.tv_nsec < NSEC_PER_SEC);
    assert(timeout.tv_nsec >= 0);

    // Put loop around pthread_cond_timedwait to handle spurious wakeups
    while (result == DEQUEUE_NOT_FOUND)
    {
      // Wait on recv condition from input thread
      error = pthread_cond_timedwait(&pkt->rx_cond, &ni->txandrx_mut, &timeout);
      if (error != 0)
      {
        if (error == ETIMEDOUT)
        {
          // Timeout
        }
        else
        {
          ec_log(EC_LOG_FATAL, "%s: error waiting on timed condition : %s\n",
                 __func__,
                 my_strerror(error, errbuf, sizeof (errbuf)));
        }
        break;
      }
      else
      {
        result = low_level_dequeue(frame, ni, handle);
        // dequeue should always return 1 (since we know the packet is there)
        // ... unless there was a spurious wakeup.
        if (result == DEQUEUE_NOT_FOUND)
        {
          ec_log(EC_LOG_FATAL, "%s: spurious wakeup : dequeue result=%d\n",
                 __func__,
                 result);
        }
      }
    }
  } //end if mayblock

  // If we didn't get the packet, release it's resources
  if (result != DEQUEUE_SUCCESS)
  {
    low_level_release(frame, ni, handle);
  }

  error = pthread_mutex_unlock(&ni->txandrx_mut);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s: error unlocking mutex : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
  }
  return (result == DEQUEUE_SUCCESS) ? true : false;
}

// Drops a packet that has been sent with ec_posix_tx()
// Returns true for success, false for errors

static bool ec_posix_drop(struct EtherCAT_Frame * frame, struct netif * ni, int handle)
{
  assert(ni != NULL);
  assert(frame != NULL);
  char errbuf[ERRBUF_LEN];

  // Find rx mutex for outstanding packet
  struct outstanding_pkt *pkt;
  int error = pthread_mutex_lock(&ni->txandrx_mut);
  assert(error == 0);
  pkt = low_level_lookup(frame, ni, handle);

  // Bad handle, double rx, etc...
  if (pkt == NULL)
  {
    pthread_mutex_unlock(&ni->txandrx_mut);
    return false;
  }

  bool success = low_level_release(frame, ni, handle);

  if (success)
  {
    ++ni->counters.sw_dropped;
  }

  error = pthread_mutex_unlock(&ni->txandrx_mut);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "%s: error unlocking mutex : %s\n",
           __func__,
           my_strerror(error, errbuf, sizeof (errbuf)));
  }

  return success;
}

// Receives a packet that has been sent with ec_posix_tx()
// Returns true for success, false for errors or timeout.

static bool ec_posix_rx(struct EtherCAT_Frame * frame, struct netif * ni, int handle)
{
  return ec_posix_rx_common(frame, ni, handle, true);
}

// Receives a packet that has been sent with ec_posix_tx(), doesn't wait
// Returns true for success, false for errors or timeout.

static bool ec_posix_rx_nowait(struct EtherCAT_Frame * frame, struct netif * ni, int handle)
{
  return ec_posix_rx_common(frame, ni, handle, false);
}

// Only attempt to send one packet

static bool ec_posix_txandrx_once(struct EtherCAT_Frame * frame, struct netif * ni)
{
  assert(ni != NULL);

  int handle = ec_posix_tx(frame, ni);
  if (handle < 0)
  {
    return false;
  }
  bool result = ec_posix_rx(frame, ni, handle);
  return result;
}

// Normal txandrx, try sending packet multiple times before giving up.

static bool ec_rtdm_txandrx(struct EtherCAT_Frame * frame, struct netif * netif)
{
  struct netif *ni = netif;
  assert(ni != NULL);
  char errbuf[ERRBUF_LEN];

  int tries = 0;
  while (tries < MAX_TRIES_TX)
  {
    bool success = ec_posix_txandrx_once(frame, ni);
    if (success)
    {
      if (tries > 0)
      {
        ec_log(EC_LOG_ERROR, "low_level_txandrx: sending/receiving failed %d times\n", tries);
      }
      return true;
    }
    int error = pthread_mutex_lock(&ni->txandrx_mut);
    assert(error == 0);
    //++ni->retries;
    error = pthread_mutex_unlock(&ni->txandrx_mut);
    assert(error == 0);
    tries++;
  }
  ec_log(EC_LOG_FATAL, "low_level_txandrx: failed %d times: Giving up\n", MAX_TRIES_TX);
  return false;
}

struct netif* init_ec(const char * interface)
{
  char errbuf[ERRBUF_LEN];

  int sock = init_socket(interface);
  if (sock < 0)
  {
    ec_log(EC_LOG_FATAL, "Socket initialisation failed\n");
    return 0;
  }

  struct netif* ni = (struct netif*) malloc(sizeof (struct netif));
  if (ni == NULL)
  {
    ec_log(EC_LOG_FATAL, "Allocating netif struct failed\n");
    return NULL;
  }
  int error = pthread_mutexattr_init(&ni->txandrx_attr);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "Initializing txandrx mutex attr failed : %s\n",
           my_strerror(error, errbuf, sizeof (errbuf)));
    free(ni);
    return NULL;
  }

  error = pthread_mutexattr_settype(&ni->txandrx_attr, PTHREAD_MUTEX_ERRORCHECK_NP);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "Setting type of mutex attr failed : %s\n",
           my_strerror(error, errbuf, sizeof (errbuf)));
    free(ni);
    return NULL;
  }

  error = pthread_mutex_init(&ni->txandrx_mut, &ni->txandrx_attr);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "Initializing txandrx mutex failed : %s\n",
           my_strerror(error, errbuf, sizeof (errbuf)));
    free(ni);
    return NULL;
  }

  int index;
  for (index = 0; index < PKT_LIST_SIZE; ++index)
  {
    if (!init_pkt(&ni->pkt_list[index]))
    {
      ec_log(EC_LOG_FATAL, "Initializing pkt %d failed\n", index);
      free(ni);
      return NULL;
    }
  }

  for (index = 0; index < BUF_LIST_SIZE; ++index)
  {
    init_buf(&ni->buf_list[index]);
  }

  ni->txandrx = ec_rtdm_txandrx;
  ni->txandrx_once = ec_posix_txandrx_once;
  ni->tx = ec_posix_tx;
  ni->rx = ec_posix_rx;
  ni->drop = ec_posix_drop;
  ni->rx_nowait = ec_posix_rx_nowait;

  ni->socket_private = sock;
  memset(&ni->counters, 0, sizeof (ni->counters));
  ni->next_pkt_index = 0;
  ni->tx_seqnum = 0;
  ni->rx_seqnum = 0xffff;

  ni->timeout_us = TIMEOUT_USEC;

  ni->unclaimed_packets = 0;

  // To differentiate between different implemenations of EML library,
  // Use a random-ish value for 2nd and 3rd bytes src MAC address
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0)
  {
    ec_log(EC_LOG_ERROR, "Gettimeofday : %s\n",
           my_strerror(error, errbuf, sizeof (errbuf)));
  }
  int r = tv.tv_sec ^ tv.tv_usec;

  //Mac-address
  ni->hwaddr[0] = 0x00;
  ni->hwaddr[2] = r;
  ni->hwaddr[4] = 0x00;
  ni->hwaddr[1] = r >> 8;
  ni->hwaddr[3] = 0x00;
  ni->hwaddr[5] = 0x00;

  // Start input thread
  ni->stop = false;
  ni->is_stopped = false;
  error = pthread_create(&ni->input_thread, NULL, low_level_input_thread_func, ni);
  if (error != 0)
  {
    ec_log(EC_LOG_FATAL, "Starting input thread failed : %s\n",
           my_strerror(error, errbuf, sizeof (errbuf)));
    free(ni);
    return NULL;
  }
  return ni;

}
