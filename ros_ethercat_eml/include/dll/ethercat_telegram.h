// $Id: ethercat_telegram.h,v 1.28 2005/06/17 13:12:38 kgad Exp $
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

 
#ifndef __ethercat_telegram_h__
#define __ethercat_telegram_h__

#include "ethercat/ethercat_defs.h"
#include "ethercat/ethercat_datastruct.h"

// Header size of telegram expressed as a number of bytes
static const size_t ETHERCAT_TELEGRAM_HEADER_SIZE = 10;
// Tail size of telegram expressed as a number of bytes
static const size_t ETHERCAT_TELEGRAM_WKC_SIZE = 2;

/// EtherCat Telegram Base class
/** This class should never be used explicitly, only use the
    interface.  Therefore its constructor is protected.  The names are
    chosen according to the Ethercat spec.
*/
class EC_Telegram : public EC_DataStruct
{
 public:
  virtual                ~EC_Telegram();

  // redefine pure virtual
  unsigned char *         dump(unsigned char * a_buffer) const;
  /// Build and check telegram from data array
  const unsigned char *   build(const unsigned char * a_buffer);

  /// Set working counter
  void                    set_wkc(EC_UINT a_wkc) { m_wkc = a_wkc; }
  /// Set index
  void                    set_idx(EC_USINT a_idx) { m_idx = a_idx; }
  /// Get working counter
  EC_UINT                 get_wkc(void) const { return m_wkc; }
  /// Get index
  EC_USINT                get_idx(void) const { return m_idx; }
  /// Get pointer to data
  const unsigned char *   get_data(void) const { return m_data; }
  /// Set data
  void                    set_data(const unsigned char * a_data){ m_data = a_data;}
  /// Get data length
  size_t                  get_datalen(void) const {return (length() - 
							   ETHERCAT_TELEGRAM_HEADER_SIZE -
							   ETHERCAT_TELEGRAM_WKC_SIZE);}
  /// Set data length
  void                    set_datalen(size_t len) { m_data_length = len  + 
						      ETHERCAT_TELEGRAM_HEADER_SIZE +
						      ETHERCAT_TELEGRAM_WKC_SIZE; }

  /// attach telegram to this one - ordering is somewhat arbitrary
  void attach(EC_Telegram *a_telegram);

  /// Pointer to next telegram
  EC_Telegram *   next;
  /// Pointer to previous telegram
  EC_Telegram *   previous;
      
  protected:
  EC_Telegram(size_t a_datasize = 0, const unsigned char * a_data = NULL);
  EC_Telegram(EC_USINT a_idx, EC_UINT a_wkc);
  EC_Telegram(size_t a_datasize, const unsigned char * a_data,
	      EC_USINT a_idx, EC_UINT a_wkc);

  EC_Telegram(const EC_Telegram& a_telegram);

  /// Dump first 6 bytes of the header (varying across different telegrams)
  /** @param a_buffer Adress to start writing
      @return Address to continu writing
  */
  virtual unsigned char * dump_header_head(unsigned char * a_buffer) const = 0;
  /// Build and check first 6 bytes of the header (varying across
  /// different telegrams)
  /** @param a_buffer Adress to start writing
      @return Address to continu writing
  */
  virtual const unsigned char * build_header_head(const unsigned char * a_buffer) = 0;

  virtual bool            check_index(const unsigned char * buffer) const;
  virtual bool            check_lennext(const unsigned char * buffer) const;

 protected:
  /// Pointer to data field
  const unsigned char *   m_data;
  // Length of data field
  //  size_t                  m_datalength;
  /// Index Field
  EC_USINT                m_idx;
  /// Working counter Field
  EC_UINT                 m_wkc;
  /// IRQ Field (currently unused)
  static const EC_UINT          m_irq;

  virtual size_t          header_length(void)const { return ETHERCAT_TELEGRAM_HEADER_SIZE; }
  virtual size_t          tail_length(void)const { return ETHERCAT_TELEGRAM_WKC_SIZE; }

 private:
  virtual const unsigned char * build_body(const unsigned char * a_buffer);
};

#endif // __ethercat_telegram_h__
