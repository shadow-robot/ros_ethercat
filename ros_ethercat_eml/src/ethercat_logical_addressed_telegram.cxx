// $Id: ethercat_logical_addressed_telegram.cxx,v 1.6 2006/02/20 15:57:33 kgad Exp $
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

#include <assert.h>
#include "ros_ethercat_eml/ethercat_logical_addressed_telegram.h"

static const uint8_t LRD = 0x0a; // Logical Read
static const uint8_t LWR = 0x0b; // Logical Write
static const uint8_t LRW = 0x0c; // Logical Read-Write

// --------------------------------------------------
// Logical Addressing Telegram
// --------------------------------------------------

#define LA_TG Logical_Addressing_Telegram

LA_TG::LA_TG(uint8_t a_idx, uint32_t a_adr,
             uint16_t a_wkc,
             uint16_t a_datalen,
             const unsigned char * a_data)
  :
  EC_Telegram(a_datalen, a_data, a_idx, a_wkc),
  m_adr(a_adr)
{
}

unsigned char *
LA_TG::dump_header_head(unsigned char * a_buffer) const
{
  a_buffer = this->dump_command_field(a_buffer);
  a_buffer = host2nw(a_buffer, m_idx);
  a_buffer = host2nw(a_buffer, m_adr);
  return a_buffer;
}

const unsigned char * LA_TG::build_header_head(const unsigned char * a_buffer)
{
  a_buffer = this->build_command_field(a_buffer);

  if (this->check_index(a_buffer) == true)
  {
    a_buffer++;
    // Read ADR (Fixme, check if not altered)
    a_buffer = nw2host(a_buffer, m_adr);
    // Leave pointer at the start of the data field
    return a_buffer;
  }
  else
    return NULL;
}

// --------------------------------------------------
// Logical Read Telegram
// --------------------------------------------------

LRD_Telegram::LRD_Telegram(uint8_t a_idx, uint32_t a_adr,
                           uint16_t a_wkc,
                           uint16_t a_datalen,
                           const unsigned char * a_data)
  :
  LA_TG(a_idx, a_adr, a_wkc, a_datalen, a_data)
{
}

unsigned char *
LRD_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, LRD);
  return a_buffer;
}

const unsigned char * LRD_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == LRD);
  return ++a_buffer;
}

// --------------------------------------------------
// Logical Write Telegram
// --------------------------------------------------

LWR_Telegram::LWR_Telegram(uint8_t a_idx, uint32_t a_adr,
                           uint16_t a_wkc,
                           uint16_t a_datalen,
                           const unsigned char * a_data)
  :
  LA_TG(a_idx, a_adr, a_wkc, a_datalen, a_data)
{
}

unsigned char *
LWR_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, LWR);
  return a_buffer;
}

const unsigned char * LWR_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == LWR);
  return ++a_buffer;
}

// --------------------------------------------------
// Logical Read Write Telegram
// --------------------------------------------------

LRW_Telegram::LRW_Telegram(uint8_t a_idx, uint32_t a_adr,
                           uint16_t a_wkc,
                           uint16_t a_datalen,
                           const unsigned char * a_data)
  :
  LA_TG(a_idx, a_adr, a_wkc, a_datalen, a_data)
{
}

unsigned char *
LRW_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, LRW);
  return a_buffer;
}

const unsigned char * LRW_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == LRW);
  return ++a_buffer;
}

