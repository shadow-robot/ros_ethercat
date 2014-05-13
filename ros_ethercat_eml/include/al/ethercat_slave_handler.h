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

#include "al/ethercat_FSM.h"
#include "al/ethercat_slave_conf.h"

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
  EtherCAT_SlaveHandler(EC_UINT a_ring_position,
			EC_UDINT a_product_code,
			EC_UDINT a_revision,
                        EC_UDINT a_serial,
			EC_FixedStationAddress a_station_address,
			EtherCAT_FMMU_Config * a_fmmu_config,
			EtherCAT_PD_Config * a_pd_config,
			EtherCAT_MbxConfig * a_mbx_config = NULL);
  /// Constructor using Slave Configuration
  EtherCAT_SlaveHandler(EC_UINT a_ring_position,
			const EtherCAT_SlaveConfig * a_sconf,
                        EC_UDINT a_serial);
  virtual ~EtherCAT_SlaveHandler();

  /// Get position in the EtherCAT logical ring
  EC_UINT get_ring_position() const {return m_ring_position; };
  /// Get serial
  EC_UDINT get_serial() const { return m_serial; };

  /// Returns and increments sequence number used for duplication mailbox write dectition
  EC_USINT get_mbx_counter();
 protected:

  /// Position in the EtherCAT logical Ring
  EC_UINT m_ring_position;

  /// Serial
  EC_UDINT m_serial;

  /// Sequence number for duplicate mailbox write detection
  EC_USINT m_mbx_counter;
};


#endif //  __ethercat_slave_handler__

