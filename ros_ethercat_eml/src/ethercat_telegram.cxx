// $Id: ethercat_telegram.cxx,v 1.28 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_telegram.h"
#include "ros_ethercat_eml/ethercat_log.h"
#include <assert.h>

// Number of bits of the Reserved and NEXT field
static const uint16_t ETHERCAT_LEN_NUM_BITS = 11;
static const uint16_t ETHERCAT_RESERVED_NUM_BITS = 4;

// --------------------------------------------------
// Class Telegram
// --------------------------------------------------

EC_Telegram::EC_Telegram(size_t a_datasize, const unsigned char * a_data)
  :
  EC_DataStruct(a_datasize +
                ETHERCAT_TELEGRAM_HEADER_SIZE +
                ETHERCAT_TELEGRAM_WKC_SIZE),
  m_data(a_data), m_idx(0), m_wkc(0)
{
  this->next = NULL;
  this->previous = NULL;
}

EC_Telegram::EC_Telegram(uint8_t a_idx, uint16_t a_wkc)
  :
  EC_DataStruct(ETHERCAT_TELEGRAM_HEADER_SIZE +
                ETHERCAT_TELEGRAM_WKC_SIZE),
  m_data(NULL),
  m_idx(a_idx), m_wkc(a_wkc)
{
  this->next = NULL;
  this->previous = NULL;
}

EC_Telegram::EC_Telegram(size_t a_datasize, const unsigned char * a_data, uint8_t a_idx,
                         uint16_t a_wkc)
  :
  EC_DataStruct(a_datasize +
                ETHERCAT_TELEGRAM_HEADER_SIZE +
                ETHERCAT_TELEGRAM_WKC_SIZE),
  m_data(a_data), m_idx(a_idx), m_wkc(a_wkc)
{
  this->next = NULL;
  this->previous = NULL;
}

EC_Telegram::EC_Telegram(const EC_Telegram& a_telegram)
  :
  EC_DataStruct(a_telegram), m_data(a_telegram.m_data)
{
  this->next = a_telegram.next;
  this->previous = a_telegram.previous;
  m_idx = a_telegram.get_idx();
  m_wkc = a_telegram.get_wkc();
}

unsigned char * EC_Telegram::dump(unsigned char * a_buffer) const
{
  // Dump first 6 bytes of header
  a_buffer = dump_header_head(a_buffer);
  // Dump remaining 4 bytes of header
  uint16_t tmp = get_datalen();
  // Implementation: NEXT = MSB
  if (this->next)
  {
    uint16_t a_chained_int = 1;
    a_chained_int = a_chained_int << (ETHERCAT_LEN_NUM_BITS + ETHERCAT_RESERVED_NUM_BITS);
    tmp |= a_chained_int;
  }
  a_buffer = host2nw(a_buffer, tmp);
  a_buffer = host2nw(a_buffer, m_irq);
  // Dump data
  memcpy(a_buffer, m_data, get_datalen());
  a_buffer += get_datalen();
  // Dump wkc
  return host2nw(a_buffer, m_wkc);
}

const unsigned char * EC_Telegram::build(const unsigned char * buffer)
{
  // Build and check first 6 bytes of header
  buffer = this->build_header_head(buffer);
  if (buffer != NULL)
  {
    // Check 4 remaining bytes of header
    if (check_lennext(buffer) == true)
      buffer += sizeof (uint16_t);
    else
      return NULL;
    // IRQ is currently not used
    uint16_t irq;
    buffer = nw2host(buffer, irq);

    // build body
    buffer = this->build_body(buffer);
    // build tail
    buffer = nw2host(buffer, m_wkc);
    return buffer;
  }
  else
    return NULL;
}

const unsigned char * EC_Telegram::build_body(const unsigned char * buffer)
{
  unsigned char * data = (unsigned char *) m_data;
  memcpy(data, buffer, get_datalen());
  return buffer + get_datalen();
}

// Check if the index is unaltered

bool EC_Telegram::check_index(const unsigned char * buffer) const
{
  const unsigned char * tmpptr = buffer;
  uint8_t index;
  tmpptr = nw2host(tmpptr, index);
  if (index == m_idx)
    return true;
  else
  {
    ec_log(EC_LOG_ERROR,
           "EC_Telegram::check_index(): Index field does not correspond with received data\n");
    return false;
  }
}

/** Check if the next field in the buffer indicates the same as the
 next field of the telegram, and if the length also corresponds */
bool EC_Telegram::check_lennext(const unsigned char * buffer) const
{
  // Unsigned int to verify if the byte sequence indicates a "next" telegram?
  uint16_t lennext = 0x0000;
  uint16_t len = 0x0000;
  uint16_t nextbit = 0x0000;

  const unsigned char * tmpptr = buffer;
  static const uint16_t NEXT_BIT = 0x8000;
  static const uint16_t LEN_BIT = 0x07ff;
  // Convert to host order
  tmpptr = nw2host(tmpptr, lennext);

  // Extract next information.  The Next bit is always the MSB of and
  // unsigned int...
  nextbit = lennext & NEXT_BIT;
  if (((nextbit == NEXT_BIT) && (next == NULL)) ||
      ((nextbit == 0x0000) && (next != NULL)))
  {
    ec_log(EC_LOG_ERROR,
           "EC_Telegram::check_lennext(): Next field does not correspond with received data\n");
    return false;
  }
  else // Extract len information
  {
    len = lennext & LEN_BIT;
    if (len != get_datalen())
    {
      ec_log(EC_LOG_ERROR,
             "EC_Telegram::check_lennext(): Len field does not correspond with received data\n");
      return false;
    }
    return true;
  }
}

// irq field is currently unused
const uint16_t EC_Telegram::m_irq = 0x0000;

/** Attach a single a_telegram after this telegram in chain
 */
void EC_Telegram::attach(EC_Telegram *a_telegram)
{
  assert(this != a_telegram);
  assert(a_telegram->next == NULL);
  assert(a_telegram->previous == NULL);
  a_telegram->next = this->next;
  a_telegram->previous = this;
  if (this->next != NULL)
  {
    this->next->previous = a_telegram;
  }
  this->next = a_telegram;
}
