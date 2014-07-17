// $Id: ethercat_device_addressed_telegram.cxx,v 1.9 2006/02/20 15:57:33 kgad Exp $
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
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
// --------------------------------------------------
// Device Addressing Telegram
// --------------------------------------------------

// Defining the different types of possible telegrams
static const uint8_t APRD = 0x01; // Autoincrement Physical read
static const uint8_t NPRD = 0x04; // Node addressed Physical read
static const uint8_t APWR = 0x02; // Autoincrement Physical write
static const uint8_t NPWR = 0x05; // Node addressed Physical write
static const uint8_t APRW = 0x03; // Autoincrement Physical read_write
static const uint8_t NPRW = 0x06; // Node addressed Physical read-write
static const uint8_t ARMW = 0x0d; // Autoincrement Physical write
// Multiple write
static const uint8_t BRD = 0x07; // Broadcast Read
static const uint8_t BWR = 0x08; // Broadcast Write

#define DA_TG Device_Addressing_Telegram

DA_TG::DA_TG(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado,
             uint16_t a_wkc,
             uint16_t a_datalen,
             const unsigned char * a_data)
  :
  EC_Telegram(a_datalen, a_data, a_idx, a_wkc),
  m_adp(a_adp),
  m_ado(a_ado)
{
}

unsigned char *
DA_TG::dump_header_head(unsigned char * a_buffer) const
{
  a_buffer = this->dump_command_field(a_buffer);
  a_buffer = host2nw(a_buffer, m_idx);
  a_buffer = host2nw(a_buffer, m_adp);
  a_buffer = host2nw(a_buffer, m_ado);
  return a_buffer;
}

const unsigned char * DA_TG::build_header_head(const unsigned char * a_buffer)
{
  a_buffer = this->build_command_field(a_buffer);

  if (this->check_index(a_buffer) == true)
  {
    a_buffer++;
    // Read ADP (could be altered)
    a_buffer = nw2host(a_buffer, m_adp);
    // Read ADO:  FIXME: could add check if this is not altered either
    a_buffer = nw2host(a_buffer, m_ado);
    // Leave pointer at the start of the data field
    return a_buffer;
  }
  else
    return NULL;
}

// --------------------------------------------------
// Auto Increment Physical Read Telegram
// --------------------------------------------------

APRD_Telegram::APRD_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
APRD_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, APRD);
  return a_buffer;
}

const unsigned char * APRD_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == APRD);
  return ++a_buffer;
}

// --------------------------------------------------
// Auto Increment Physical Write Telegram
// --------------------------------------------------

APWR_Telegram::APWR_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
APWR_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, APWR);
  return a_buffer;
}

const unsigned char * APWR_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == APWR);
  return ++a_buffer;
}

// --------------------------------------------------
// Auto Increment Physical Read Write Telegram
// --------------------------------------------------

APRW_Telegram::APRW_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
APRW_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, APRW);
  return a_buffer;
}

const unsigned char * APRW_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == APRW);
  return ++a_buffer;
}

// --------------------------------------------------
// Broadcast Write Telegram
// --------------------------------------------------

BWR_Telegram::BWR_Telegram(uint8_t a_idx, uint16_t a_ado,
                           uint16_t a_wkc,
                           uint16_t a_datalen,
                           const unsigned char * a_data)
  :
  DA_TG(a_idx, 0x0000, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
BWR_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, BWR);
  return a_buffer;
}

const unsigned char * BWR_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == BWR);
  return ++a_buffer;
}

// --------------------------------------------------
// Broadcast Read Telegram
// --------------------------------------------------

BRD_Telegram::BRD_Telegram(uint8_t a_idx, uint16_t a_ado,
                           uint16_t a_wkc,
                           uint16_t a_datalen,
                           const unsigned char * a_data)
  :
  DA_TG(a_idx, 0x0000, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
BRD_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, BRD);
  return a_buffer;
}

const unsigned char * BRD_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == BRD);
  return ++a_buffer;
}

// --------------------------------------------------
// Node Addressed Physical Write Telegram
// --------------------------------------------------

NPWR_Telegram::NPWR_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
NPWR_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, NPWR);
  return a_buffer;
}

const unsigned char * NPWR_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == NPWR);
  return ++a_buffer;
}

// --------------------------------------------------
// Node Addressed Physical Read Telegram
// --------------------------------------------------

NPRD_Telegram::NPRD_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
NPRD_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, NPRD);
  return a_buffer;
}

const unsigned char * NPRD_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == NPRD);
  return ++a_buffer;
}

// --------------------------------------------------
// Node Addressed Physical Read Write Telegram
// --------------------------------------------------

NPRW_Telegram::NPRW_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
NPRW_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, NPRW);
  return a_buffer;
}

const unsigned char * NPRW_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == NPRW);
  return ++a_buffer;
}

// --------------------------------------------------
// Auto Increment Physical Read Multiple Write Telegram
// --------------------------------------------------

ARMW_Telegram::ARMW_Telegram(uint8_t a_idx, uint16_t a_adp, uint16_t a_ado, uint16_t a_wkc,
                             uint16_t a_datalen,
                             const unsigned char * a_data)
  :
  DA_TG(a_idx, a_adp, a_ado, a_wkc, a_datalen, a_data)
{
}

unsigned char *
ARMW_Telegram::dump_command_field(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, ARMW);
  return a_buffer;
}

const unsigned char * ARMW_Telegram::build_command_field(const unsigned char * a_buffer)
{
  assert(a_buffer[0] == ARMW);
  return ++a_buffer;
}
