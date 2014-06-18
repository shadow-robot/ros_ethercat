// $Id: ethercat_mbx.cxx,v 1.8 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_mbx.h"
#include <assert.h>

EC_MbxMsgHdr::EC_MbxMsgHdr(const unsigned char * a_buffer)
  :
  EC_DataStruct(EC_MBXMSG_HDR_SIZE), m_address(a_buffer + sizeof (m_length))
{
  a_buffer = nw2host(a_buffer, m_length);
  a_buffer += m_address.length();
  uint8_t priority;
  a_buffer = nw2host(a_buffer, priority);
  m_priority = priority >> 6; // bitshifting 6 bits
  uint8_t msg_type;
  a_buffer = nw2host(a_buffer, msg_type);
  msg_type &= 0x7; // Only last 3 bits should last
  assert(msg_type <= EC_FoE);
  m_type = (ECMbxMsgType) msg_type;
}

unsigned char *
EC_MbxMsgHdr::dump(unsigned char * a_buffer) const
{
  a_buffer = host2nw(a_buffer, m_length);
  a_buffer = m_address.dump(a_buffer);

  uint8_t priority = m_priority << 6;
  a_buffer = host2nw(a_buffer, priority);

  a_buffer = host2nw(a_buffer, m_type);
  return a_buffer;
}

// ==================================================

EtherCAT_MbxMsg::EtherCAT_MbxMsg(const unsigned char * a_buffer)
  :
  m_hdr(a_buffer)
{
  a_buffer += EC_MBXMSG_HDR_SIZE;
  m_MbxMsgdata = a_buffer;
}

unsigned char *
EtherCAT_MbxMsg::dump_data(unsigned char * a_buffer) const
{
  memcpy(a_buffer, m_MbxMsgdata, m_hdr.m_length);
  return (a_buffer + m_hdr.m_length);
}

unsigned char *
EtherCAT_MbxMsg::dump(unsigned char * a_buffer) const
{
  a_buffer = m_hdr.dump(a_buffer);
  a_buffer = dump_data(a_buffer);
  return a_buffer;
}

// ==================================================

EC_CoE_Hdr::EC_CoE_Hdr(const unsigned char * a_buffer)
  :
  EC_DataStruct(EC_MBXMSG_COE_HDR_SIZE)
{
  uint16_t hdr;
  a_buffer = nw2host(a_buffer, hdr);
  // FIXME Number Hi and Number Lo not yet implemented
  hdr = hdr >> 12;
  hdr &= 0xf;
  assert(hdr <= CANopen_SDOInformation);
  m_service = (CANopenService) hdr;
}

unsigned char *
EC_CoE_Hdr::dump(unsigned char * a_buffer) const
{
  uint16_t hdr = m_service;
  a_buffer = host2nw(a_buffer, hdr);
  return a_buffer;
}

// ==================================================

EtherCAT_CoE_MbxMsg::EtherCAT_CoE_MbxMsg(unsigned char * a_buffer)
  :
  EtherCAT_MbxMsg(a_buffer), m_CoE_Hdr(a_buffer + EC_MBXMSG_HDR_SIZE)
{
  a_buffer += (EC_MBXMSG_COE_HDR_SIZE + EC_MBXMSG_HDR_SIZE);
  m_MbxMsgdata = a_buffer;
}

unsigned char *
EtherCAT_CoE_MbxMsg::dump(unsigned char * a_buffer) const
{
  a_buffer = m_hdr.dump(a_buffer);
  a_buffer = m_CoE_Hdr.dump(a_buffer);
  a_buffer = this->dump_data(a_buffer);
  return a_buffer;
}

