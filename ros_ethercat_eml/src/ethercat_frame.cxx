// $Id: ethercat_frame.cxx,v 1.18 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_log.h"

int framedump(const struct EtherCAT_Frame * frame,
              unsigned char * buffer,
              size_t bufferlength)
{
  uint16_t length = 0;
  if ((length = frame->length()) <= bufferlength)
  {
    frame->dump(buffer);
  }
  else
  {
    return 0;
  }
  return length;
}

int framebuild(struct EtherCAT_Frame * frame,
               const unsigned char * buffer)
{
  return (frame->build(buffer));
}

// --------------------------------------------------
// EC_Frame
// --------------------------------------------------

EC_Frame::EC_Frame()
  :
  m_telegram(NULL)
{
}
;

EC_Frame::EC_Frame(EC_Telegram * a_telegram)
  :
  m_telegram(a_telegram)
{
}
;

size_t
EC_Frame::body_length(void) const
{
  int result = 0;
  if (m_telegram)
  {
    EC_Telegram * tg = m_telegram;
    result += tg->length();
    while (tg->next != NULL)
    {
      tg = tg->next;
      result += tg->length();
    }
  }
  return result;
}

unsigned char *
EC_Frame::dump(unsigned char * a_buffer) const
{
  a_buffer = dump_header(a_buffer);
  if (m_telegram)
  {
    EC_Telegram * tg = m_telegram;
    a_buffer = tg->dump(a_buffer);
    while (tg->next != NULL)
    {
      tg = tg->next;
      a_buffer = tg->dump(a_buffer);
    }
  }
  return a_buffer;
}

int
EC_Frame::build(const unsigned char * a_buffer)
{
  if (this->check_header(a_buffer) == true)
  {
    a_buffer += ETHERCAT_ETHERNET_FRAME_HEADER_SIZE;
    // a_buffer now points to the first telegram
    // current telegram in frame, initialised to first telegram
    EC_Telegram * tg = this->get_telegram();
    while (tg)
    {
      if ((a_buffer = tg->build(a_buffer)) == NULL)
        return -1;
      else
        tg = tg->next;
    }
    return 0;
  }
  else
    return -1;
}

// --------------------------------------------------
// EC_Ethernet_Frame
// --------------------------------------------------

EC_Ethernet_Frame::EC_Ethernet_Frame(EC_Telegram * a_telegram)
  :
  EC_Frame(a_telegram)
{
}

unsigned char *
EC_Ethernet_Frame::dump_header(unsigned char * a_buffer) const
{
  // Implementation: LSB = length, then Reserved, then MSB = type
  uint16_t tmp = 0x01; // Type = Ethercat command
  tmp = tmp << 12;
  tmp |= this->body_length();

  a_buffer = host2nw(a_buffer, tmp);
  return a_buffer;
}

bool
EC_Ethernet_Frame::check_header(const unsigned char * a_buffer) const
{
  const unsigned char * dataptr = a_buffer;
  uint16_t frame_header;
  dataptr = nw2host(dataptr, frame_header);

  uint16_t ec_command = 0x1000;
  if (ec_command == (frame_header & ec_command))
  {
    uint16_t bodylength = 0x07ff;
    if (this->body_length() == (size_t) (frame_header & bodylength))
      return true;
  }
  ec_log(EC_LOG_ERROR, "building frame: checkheader failed\n");
  return false;
}

