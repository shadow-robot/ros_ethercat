// $Id: ethercat_process_data.h,v 1.8 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_pd__
#define __ethercat_pd__

// forward declarations
class EC_Logic;
class EtherCAT_DataLinkLayer;

#include "dll/ethercat_logical_addressed_telegram.h"
#include "dll/ethercat_frame.h"

/// EtherCAT Process Data buffer
/** @note The current implementation starts passing PD as soon as *one* of the
    slaves indicated it arrived in its safe-operational state by using
    the EtherCAT_PD_Buffer::start() method.  The advantage is that an application could remain running.
    If a slave follows the spec, it won't process PD unless it is in
    its SafeOp or Op state, so this should harm...
    Nevertheless, changing this behaviour into full spec compliance
    only requires changing 2 lines of code.  See the .cxx file.
*/
class EtherCAT_PD_Buffer
{
  friend class EC_ESM_Ops;
  
 public:
  /// Singleton
  static EtherCAT_PD_Buffer * instance();

  /// Send some process data
  /** @param datalen number of bytes that should be set
      @param data pointer to data array.  Data is read, transmitted,
      and (if the operation succeeded) the received data is put in the
      data array
      @return true if msg got true
  */
  bool txandrx(size_t datalen, unsigned char * data);

 protected:  
  /// Start transmitting process data
  void start();
  /// Stop transmitting process data
  void stop();

  /// Constructor
  EtherCAT_PD_Buffer();
  virtual ~EtherCAT_PD_Buffer();
  
 private:
  /// Pointer to EC_Logic
  EC_Logic * m_logic_instance;
  /// Pointer to DLL instance
  EtherCAT_DataLinkLayer * m_dll_instance;
  /// This class is a singleton
  static EtherCAT_PD_Buffer * m_instance;

  /// See note in class definition.
  unsigned int m_is_running;

  /// Process data can be divided over MAX_CHUCKS packets 
  /// of upto CHUNK_SIZE bytes
  static const unsigned MAX_CHUNKS=4;
  static const unsigned CHUNK_SIZE=1486;

  /// Telegram(s) to be sent
  LRW_Telegram *m_lrw_telegram[MAX_CHUNKS];
  /// EtherCAT frame(s) to be sent
  EC_Ethernet_Frame *m_lrw_frame[MAX_CHUNKS];
};

#endif
