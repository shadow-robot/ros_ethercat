// $Id: ethercat_master.cxx,v 1.28 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_master.h"
#include "ros_ethercat_eml/ethercat_AL.h"
#include "ros_ethercat_eml/ethercat_router.h"
#include "ros_ethercat_eml/ethercat_slave_conf.h"
#include "ros_ethercat_eml/ethercat_slave_memory.h"
#include "ros_ethercat_eml/ethercat_dll.h"
#include "ros_ethercat_eml/ethercat_telegram.h"
#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
#include "ros_ethercat_eml/ethercat_logical_addressed_telegram.h"

EtherCAT_SlaveHandler *EtherCAT_Master::get_slave_handler(EC_FixedStationAddress address)
{
  return m_al_instance->get_slave_handler(address);
}

bool EtherCAT_Master::txandrx_PD(size_t datalen, unsigned char * data)
{
  return m_pdbuf_instance->txandrx(datalen, data);
}


