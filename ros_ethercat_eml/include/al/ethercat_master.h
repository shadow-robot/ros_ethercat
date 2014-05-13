// $Id: ethercat_master.h,v 1.25 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_master__
#define __ethercat_master__

// Forward declarations
class EtherCAT_AL;
class EtherCAT_SlaveHandler;
class EtherCAT_Router;
class EtherCAT_PD_Buffer;
class EC_Logic;
class EtherCAT_DataLinkLayer;

#include "al/ethercat_mbx.h"
#include "al/ethercat_process_data.h"
#include "dll/ethercat_slave_memory.h"

/// EtherCAT Master instance
class EtherCAT_Master
{
 public:
  /// This class is a singleton
  static EtherCAT_Master * instance();
  /// Destructor
  virtual ~EtherCAT_Master();

  /// Get Slave Handler
  /** @return Pointer to slavehandler if found in the network, or NULL
      otherwise 
  */
  EtherCAT_SlaveHandler * get_slave_handler(EC_FixedStationAddress address);

  /// Send Process data
  /** @param datalen number of bytes that should be set
      @param data pointer to data array.  Data is read, transmitted,
      and (if the operation succeeded) the received data is put in the 
      data array.  This happens synchronously with the method call.
      @return true if msg got true
  */
  bool txandrx_PD(size_t datalen,
		  unsigned char * data);

 protected:
  /// Constructor (protected)
  EtherCAT_Master();

 private:
  /// Master instance
  static EtherCAT_Master * m_instance;
  /// Pointer to m_AL_instance
  EtherCAT_AL * m_al_instance;
  /// Pointer to router instance
  EtherCAT_Router * m_router_instance;
  /// Pointer to process data buffer instance
  EtherCAT_PD_Buffer * m_pdbuf_instance;

  /* If master also has slave functionality, it has its own mailbox
     and its own address, so slaves can post msgs to it.
  EtherCAT_Mbx m_mbx;
  EC_FixedStationAddress m_address;
  */

  /// Pointer to logic instance
  EC_Logic * m_logic_instance;
  /// Pointer to DLL instance
  EtherCAT_DataLinkLayer * m_dll_instance;
};

#endif
