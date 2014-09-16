// $Id: ethercat_slave_handler.cxx,v 1.19 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_slave_handler.h"
#include <assert.h>

EtherCAT_SlaveHandler::EtherCAT_SlaveHandler(uint16_t a_ring_position,
                                             uint32_t a_product_code,
                                             uint32_t a_revision,
                                             uint32_t a_serial,
                                             EC_FixedStationAddress a_station_address,
                                             EtherCAT_FMMU_Config * a_fmmu_config,
                                             EtherCAT_PD_Config * a_pd_config,
                                             EtherCAT_MbxConfig * a_mbx_config,
                                             EtherCAT_DataLinkLayer *_m_dll_instance,
                                             EC_Logic *_m_logic_instance,
                                             EtherCAT_PD_Buffer *_m_pdbuf_instance) :
  EC_ESM(this,
         _m_dll_instance,
         _m_logic_instance,
         _m_pdbuf_instance),
  EtherCAT_SlaveConfig(a_product_code,
                       a_revision,
                       a_station_address,
                       a_fmmu_config,
                       a_pd_config,
                       a_mbx_config),
  m_ring_position(a_ring_position),
  m_serial(a_serial),
  m_mbx_counter(0)
{
}

EtherCAT_SlaveHandler::EtherCAT_SlaveHandler(uint16_t a_ring_position,
                                             EtherCAT_SlaveConfig * a_sconf,
                                             uint32_t a_serial,
                                             EtherCAT_DataLinkLayer *_m_dll_instance,
                                             EC_Logic *_m_logic_instance,
                                             EtherCAT_PD_Buffer *_m_pdbuf_instance) :
  EC_ESM(this,
         _m_dll_instance,
         _m_logic_instance,
         _m_pdbuf_instance),
  EtherCAT_SlaveConfig(*a_sconf),
  m_ring_position(a_ring_position),
  m_serial(a_serial),
  m_mbx_counter(0)
{
}

uint8_t EtherCAT_SlaveHandler::get_mbx_counter()
{
  m_mbx_counter = 1 + (m_mbx_counter % 7);
  assert(m_mbx_counter > 0);
  assert(m_mbx_counter <= 7);
  return m_mbx_counter;
}

