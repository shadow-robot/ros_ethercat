// $Id: ethercat_logical_addressed_telegram.h,v 1.8 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_logical_addressed_telegram_h__
#define __ethercat_logical_addressed_telegram_h__

#include "ethercat_telegram.h"

/// EtherCAT telegram for all _logical addressing_ modes
/** EtherCAT telegram for all _logical addressing_ modes, that is
    logical read (LRD), logical write (LWR), and logical Read-Write (LRW)    
*/
class Logical_Addressing_Telegram : public EC_Telegram {
 public:
  /// Constructor
  /** @param a_idx index
      @param a_adr logical address (4 bytes)
      @param a_wkc working counter
      @param a_datalen data_length
      @param a_data data
  */
  Logical_Addressing_Telegram(EC_USINT a_idx=0x00, 
			      EC_UDINT a_adr=0x00000000, 
			      EC_UINT a_wkc=0x0000, 
			      EC_UINT a_datalen=0x0000, 
			      const unsigned char * a_data = NULL);

  virtual                ~Logical_Addressing_Telegram();

  /// Set address field
  /** @param a_adr 4 byte logical address
   */
  void                    set_adr(EC_UDINT a_adr) { m_adr = a_adr; }
  /// Get address field
  /** @return 4 byte logical address
   */
  EC_UDINT                get_adr() const { return m_adr;}
  
 protected:
  virtual unsigned char * dump_header_head(unsigned char * a_buffer) const;
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const = 0;
  
  virtual const unsigned char * build_header_head(const unsigned char * a_buffer);
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer) = 0;
  
 private:
  EC_UDINT                 m_adr;
};

/// Logical Read Telegram
class LRD_Telegram : public Logical_Addressing_Telegram {
 public:
  /// Constructor
  /** @param a_idx index
      @param a_adr logical address
      @param a_wkc working counter
      @param a_datalen data_length
      @param a_data data to be read
  */
  LRD_Telegram(EC_USINT a_idx, EC_UDINT a_adr,
	       EC_UINT a_wkc, EC_UINT a_datalen,
	       const unsigned char * a_data);
			    
  virtual                ~LRD_Telegram();

 protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Logical Write Telegram
class LWR_Telegram : public Logical_Addressing_Telegram {
 public:
  /// Constructor
  /** @param a_idx index
      @param a_adr logical address
      @param a_wkc working counter
      @param a_datalen data_length
      @param a_data data to be written
  */
  LWR_Telegram(EC_USINT a_idx, EC_UDINT a_adr,
	       EC_UINT a_wkc, EC_UINT a_datalen,
	       const unsigned char * a_data);
			    
  virtual                ~LWR_Telegram();

 protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Logical Read Write Telegram
class LRW_Telegram : public Logical_Addressing_Telegram {
 public:
  /// Constructor
  /** @param a_idx index
      @param a_adr logical address
      @param a_wkc working counter
      @param a_datalen data_length
      @param a_data data to be written and read
  */
  LRW_Telegram(EC_USINT a_idx, EC_UDINT a_adr,
	       EC_UINT a_wkc, EC_UINT a_datalen,
	       const unsigned char * a_data);
			    
  virtual                ~LRW_Telegram();

 protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

#endif // __ethercat_logical_addressed_telegram_h__
