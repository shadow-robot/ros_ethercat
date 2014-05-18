// $Id: ethercat_router.cxx,v 1.19 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_router.h"
#include "ros_ethercat_eml/ethercat_mbx.h"
#include "ros_ethercat_eml/ethercat_slave_handler.h"
#include "ros_ethercat_eml/ethercat_AL.h"
#include "ros_ethercat_eml/ethercat_master.h"
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_dll.h"

EtherCAT_Router * EtherCAT_Router::m_instance = 0;

EtherCAT_Router *
EtherCAT_Router::instance()
{
  if (!m_instance)
  {
    m_instance = new EtherCAT_Router();
  }
  return m_instance;
}

// al_instance cannot be initiated right now, since this results in a
// circular instantiation al_instance calls router_instance etc.
EtherCAT_Router::EtherCAT_Router()
:
    m_al_instance(NULL), m_is_running(0)
{
  // get pointer to DLL and logic
  m_dll_instance = EtherCAT_DataLinkLayer::instance();
  m_logic_instance = EC_Logic::instance();
}

void
EtherCAT_Router::start()
{
  // Only instantiate here...
  m_al_instance = EtherCAT_AL::instance();
  ++m_is_running;
}

void
EtherCAT_Router::stop()
{
  if (EtherCAT_Router::m_is_running > 0)
    --EtherCAT_Router::m_is_running;
  else
  ec_log(EC_LOG_INFO, "EtherCAT_Router already stopped...\n");
}

void
EtherCAT_Router::route() const
{
  if (EtherCAT_Router::m_is_running != 0)
  {
    // ec_log(EC_LOG_INFO, "EtherCAT_Router::Routing\n");
    EtherCAT_SlaveHandler * sh;
    for (unsigned int i = 0; i < m_al_instance->get_num_slaves(); i++)
    {
      sh = m_al_instance->m_slave_handler[i];
      // don't use return value?
      check_mbx(sh);
    }
  }
}

bool
EtherCAT_Router::check_mbx(const EtherCAT_SlaveHandler * sh) const
                           {
  if (sh->is_complex())
  {
    // SM0 is for M->S communication, SM1 for S->M
    const uint16_t datalen = sh->get_mbx_config()->SM1.Length;
    unsigned char mbx_data[datalen];
    NPRD_Telegram chk_mbx_tg(m_logic_instance->get_idx(),
                             sh->get_station_address(),
                             sh->get_mbx_config()->SM1.PhysicalStartAddress,
                             m_logic_instance->get_wkc(),
                             datalen,
                             mbx_data);
    EC_Ethernet_Frame chk_mbx_frame(&chk_mbx_tg);
    if (m_dll_instance->txandrx(&chk_mbx_frame))
    {
      // If slave posted something, wkc has increased
      if (chk_mbx_tg.get_wkc() == 0x1)
      {
        EtherCAT_MbxMsg msg(chk_mbx_tg.get_data());
        return (post_mbxmsg(&msg, sh));
      }
    }
    else
    {
      ec_log(EC_LOG_ERROR, "Router: Error checking mbx\n");
      return false;
    }
  }
  return true;
}

bool
EtherCAT_Router::post_mbxmsg(EtherCAT_MbxMsg * msg, const EtherCAT_SlaveHandler * from_sh) const
                             {
  EC_FixedStationAddress dest_addr = msg->m_hdr.m_address;
  EtherCAT_SlaveHandler * dest_sh = m_al_instance->get_slave_handler(dest_addr);
  if (dest_sh->is_complex())
  {
    // Check if MBX sizes correspond...
    const uint16_t datalen = dest_sh->get_mbx_config()->SM0.Length;
    unsigned char mbx_data[datalen];
    if (dest_sh->get_mbx_config()->SM0.Length == from_sh->get_mbx_config()->SM1.Length)
    {
      // Alter Header:  Include source instead of destination
      msg->m_hdr.m_address = from_sh->get_station_address();
      msg->dump(mbx_data);
      NPWR_Telegram put_mbx_tg(m_logic_instance->get_idx(),
                               dest_addr,
                               from_sh->get_mbx_config()->SM0.PhysicalStartAddress,
                               m_logic_instance->get_wkc(),
                               dest_sh->get_mbx_config()->SM1.Length,
                               mbx_data);
      EC_Ethernet_Frame put_mbx_frame(&put_mbx_tg);
      bool succeed = false;
      // According to the spec, we have to keep trying until
      // the slave accepts the command (could be \inf loop!).
      while (succeed == false)
        succeed = m_dll_instance->txandrx(&put_mbx_frame);
      return succeed;
    }
    else
    {
      ec_log(EC_LOG_ERROR,
             "Router::post_mbxmsg() error: SM sizes of source and destination do not match...!!\n");
      return false;
    }
  }
  else
  {
    ec_log(EC_LOG_ERROR, "Router Error: Destination address of MbxMsg is not a complex slave!!\n");
    return false;
  }
}

/// C-interface to perform the routing
void route_msgs(void)
{
  static EtherCAT_Router * EC_Router = EtherCAT_Router::instance();
  EC_Router->route();
  // ec_log(EC_LOG_INFO, "Routing Mbx msgs\n");
}

