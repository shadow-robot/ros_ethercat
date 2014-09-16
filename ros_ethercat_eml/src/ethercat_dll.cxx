// $Id: ethercat_dll.cxx,v 1.30 2006/02/20 15:57:33 kgad Exp $
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

#include <string.h>
#include "ros_ethercat_eml/ethercat_log.h"
#include "ros_ethercat_eml/ethercat_dll.h"
#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
#include "ros_ethercat_eml/ethercat_logical_addressed_telegram.h"
#include "ros_ethercat_eml/netif.h"

#include "unistd.h"

bool EtherCAT_DataLinkLayer::txandrx(EtherCAT_Frame * a_frame)
{
  bool succeed = m_if->txandrx(a_frame, m_if);
  if (!succeed)
    ROS_DEBUG("DLL::txandrx() Error");
  return succeed;
}

int EtherCAT_DataLinkLayer::tx(EtherCAT_Frame * a_frame)
{
  int handle = m_if->tx(a_frame, m_if);
  if (handle < 0)
    ROS_DEBUG("DLL::tx Error");
  return handle;
}

bool EtherCAT_DataLinkLayer::rx(EtherCAT_Frame * a_frame, int a_handle)
{
  bool succeed = m_if->rx(a_frame, m_if, a_handle);
  if (!succeed)
    ROS_DEBUG("DLL::rx Error");
  return succeed;
}

void EtherCAT_DataLinkLayer::attach(struct netif * netif)
{
  m_if = netif;
}

