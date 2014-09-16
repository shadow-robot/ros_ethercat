// $Id: ethercat_slave_handler.h,v 1.25 2006/02/20 15:57:33 kgad Exp $
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

#ifndef __ethercat_slave_handler__
#define __ethercat_slave_handler__

#include "ros_ethercat_eml/ethercat_FSM.h"
#include "ros_ethercat_eml/ethercat_slave_conf.h"

// forward declaration
class EtherCAT_AL;

/// EtherCAT Slave Handler

class EtherCAT_SlaveHandler : public EC_ESM, public EtherCAT_SlaveConfig
{
  // Being friendly
  friend class EC_ESM_Ops;
  friend class EtherCAT_Router;
  friend class EtherCAT_AL;

public:
  /// Constructor
  /** @param a_ring_position position in the EtherCAT ring
   @param a_product_code product code of the slave
   @param a_revision revision of the slave
   @param a_station_address address of the station as given in
   config file
   @param a_fmmu_config pointer to configuration of fmmus as
   created when parsing config file
   @param a_pd_config pointer to configuration of SM for process
   data
   @param a_mbx_config MBX configuration if this slave is complex.
   Default argument is NULL for simple slaves
   */
  EtherCAT_SlaveHandler(uint16_t a_ring_position,
                        uint32_t a_product_code,
                        uint32_t a_revision,
                        uint32_t a_serial,
                        EC_FixedStationAddress a_station_address,
                        EtherCAT_FMMU_Config * a_fmmu_config,
                        EtherCAT_PD_Config * a_pd_config,
                        EtherCAT_MbxConfig * a_mbx_config,
                        EtherCAT_DataLinkLayer *_m_dll_instance,
                        EC_Logic *_m_logic_instance,
                        EtherCAT_PD_Buffer *_m_pdbuf_instance);
  /// Constructor using Slave Configuration
  EtherCAT_SlaveHandler(uint16_t a_ring_position,
                        EtherCAT_SlaveConfig * a_sconf,
                        uint32_t a_serial,
                        EtherCAT_DataLinkLayer *_m_dll_instance,
                        EC_Logic *_m_logic_instance,
                        EtherCAT_PD_Buffer *_m_pdbuf_instance);

  /// Get position in the EtherCAT logical ring
  uint16_t get_ring_position() const
  {
    return m_ring_position;
  }
  /// Get serial
  uint32_t get_serial() const
  {
    return m_serial;
  }

  /// Returns and increments sequence number used for duplication mailbox write dectition
  uint8_t get_mbx_counter();
protected:

  /// Position in the EtherCAT logical Ring
  uint16_t m_ring_position;

  /// Serial
  uint32_t m_serial;

  /// Sequence number for duplicate mailbox write detection
  uint8_t m_mbx_counter;
};

#endif //  __ethercat_slave_handler__
