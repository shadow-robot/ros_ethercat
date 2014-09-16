// $Id: netif.h,v 1.13 2006/02/20 15:57:33 kgad Exp $
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

// $Id: netif.h,v 1.13 2006/02/20 15:57:33 kgad Exp $
#ifndef __posix_netif_h__
#define __posix_netif_h__

// Forward declarations
struct EtherCAT_Frame;
struct netif;

#include <stdint.h>
#include <stdlib.h>

#define ETHERCAT_DEVICE_NAME_MAX_LENGTH 256
// Size of MAC addresses expressed as a number of bytes
#define MAC_ADDRESS_SIZE 6

void if_attach(struct netif * netif);
int framedump(const struct EtherCAT_Frame * frame, unsigned char * buffer, size_t bufferlength);
int framebuild(struct EtherCAT_Frame * frame, const unsigned char * buffer);

// Number of outstanding packets to keep track of
#define PKT_LIST_SIZE 128

// Number of buffers to hold received packets
#define BUF_LIST_SIZE 16

// Should be < number of RX buffers
#define MAX_UNCLAIMED_PACKETS (BUF_LIST_SIZE-1)

// buffer to hold received packets

struct pkt_buf
{
  // True if the buffer is being used
  bool is_free;
  // buffer for to store ethernet message
  // more than enough to hold any type of message
  unsigned char data[2000];
};

// record of packet that has been sent but.
//  1. has not received reply
//      .... OR ....
//  2. has not been claimed by higher level software

struct outstanding_pkt
{
  // whether this record is holding an outstanding packet or not
  bool is_free;

  // pointer to received packet buffer
  // If this is NULL if the packet has not been received
  struct pkt_buf *buf;

  // The source MAC address is used to keep track of
  // which packet belongs where
  uint8_t ether_shost[MAC_ADDRESS_SIZE];

  // EtherCAT_Frame that was use to generate the sent packet...
  // The exact same frame must be used to pickup the packet
  struct EtherCAT_Frame * frame;

  // To allow synchronization between input thread and rx function
  //pthread_mutex_t rx_mut;
  //pthread_mutexattr_t rx_attr;
  pthread_cond_t rx_cond;

  // Temember when packet was sent
  struct timespec tx_time;
};

struct netif_counters
{
  uint64_t sent;
  uint64_t received;
  uint64_t collected;
  uint64_t dropped;
  uint64_t tx_error;
  uint64_t tx_net_down;
  uint64_t tx_would_block;
  uint64_t tx_no_bufs;
  uint64_t tx_full;
  uint64_t rx_runt_pkt;
  uint64_t rx_not_ecat;
  uint64_t rx_other_eml;
  uint64_t rx_bad_index;
  uint64_t rx_bad_seqnum;
  uint64_t rx_dup_seqnum;
  uint64_t rx_dup_pkt;
  uint64_t rx_bad_order;
  uint64_t rx_late_pkt; // Count of all late packets
  uint64_t sw_dropped; // packets that were dropped by software (for testing purposes)
  uint64_t rx_late_pkt_rtt_us; // Round trip time (in microseconds) of last late packet arrival
  uint64_t rx_late_pkt_rtt_us_sum; // Sum of rtt's of all late packets (for calculating average)
};

/// Generic ethercat interface towards lower level drivers.

/** It should be readily re-implemented for different OSes such as
 RTAI, Linux, ...     etc. (For the ease of porting the interface
 is in C).
 */
struct netif
{
  /// Transmit and receive an EtherCAT frame
  /** Implemented for ecos in low_level_txandrx() in
   packages/io/eth/current/src/ethercatmaster/eth_drv.c
   and mapped in eth_drv_init()
   */
  bool (*txandrx)(struct EtherCAT_Frame * frame, struct netif * netif);

  /// Transmit and receive an EtherCAT frame - only attempt to send
  /// frame once
  bool (*txandrx_once)(struct EtherCAT_Frame * frame, struct netif * netif);

  /// Transmit frame
  /** Negative return value is an error code, positive and zero is a
   handle
   */
  int (*tx)(struct EtherCAT_Frame * frame, struct netif * netif);

  /// Receive frame
  /** May block, returns true if correct frame is received
   */
  bool (*rx)(struct EtherCAT_Frame * frame, struct netif * netif, int handle);

  /// Drop frame
  /** Like rx(), but does not fill in frame with result, useful for testing
   */
  bool (*drop)(struct EtherCAT_Frame * frame, struct netif * netif, int handle);

  /// Receive frame
  /** Will not block, returns true if correct frame is received
   */
  bool (*rx_nowait)(struct EtherCAT_Frame * frame, struct netif * netif, int handle);

  /// The MAC address
  unsigned char hwaddr[MAC_ADDRESS_SIZE];
  /// File descriptor of the socket
  int socket_private;

  /// Counters for certain types of events (packet drops, invalid packets, etc)
  struct netif_counters counters;

  /// Sequence number to put on next sent packet
  unsigned tx_seqnum;

  /// Sequence number of more recently received packet
  unsigned rx_seqnum;

  /// Outstanding pkt slot to use from next tx
  unsigned next_pkt_index;

  /// List of outstanding packets
  /// (packets that have been tx'ed but not rx'ed)
  struct outstanding_pkt pkt_list[PKT_LIST_SIZE];

  // Number of tx'd packets that have not been rx'd yet
  unsigned unclaimed_packets;

  /// List of buffers used for packet reception
  struct pkt_buf buf_list[BUF_LIST_SIZE];

  // For thread safety: txandrx() can be called from multiple threads...
  pthread_mutex_t txandrx_mut;
  pthread_mutexattr_t txandrx_attr;

  // For input thread -- if it is used
  pthread_t input_thread;
  volatile bool stop;
  volatile bool is_stopped;

  // Timeout for receive in microseconds.
  int timeout_us;

  // private variable
  void* private_data;
};

#endif // __netif_h__
