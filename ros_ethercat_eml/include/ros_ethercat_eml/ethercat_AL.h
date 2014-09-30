// $Id: ethercat_AL.h,v 1.15 2006/02/20 15:57:33 kgad Exp $
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

#ifndef __ethercat_al__
#define __ethercat_al__

#include "ros_ethercat_eml/ethercat_defs.h"

// forward declaration
class EtherCAT_DataLinkLayer;
class EtherCAT_SlaveHandler;
class EC_FixedStationAddress;
class EtherCAT_PD_Buffer;

/// Convert ringpos2adp in case of autoincrement addressing;
/** @param ringpos Position in the ring (counting starts from 0)
 @return adp
 */
inline uint16_t ringpos2adp(uint16_t ringpos)
{
  return (uint16_t) (0x0000 - ringpos);
}

/// Convert adp2ringpos in case of autoincrement addressing;
/** @param adp Auto increment address
 @return ringpos
 */
inline uint16_t adp2ringpos(uint16_t adp)
{
  return (uint16_t) (0x0000 - adp);
}

/// EtherCAT "logic"

/** @todo search better name for this
 */
class EC_Logic
{
public:
  EC_Logic() : m_wkc(0), m_idx(0)
  {
  }

  /// Get current idx
  uint8_t get_idx();
  /// Get wkc (normally always zero, not truly necessary)
  uint16_t get_wkc();

  /// Working Counter
  const uint16_t m_wkc;
  /// Index (Increased with each frame the master sends)
  uint8_t m_idx;
};
inline uint8_t EC_Logic::get_idx()
{
  return m_idx++;
}
inline uint16_t EC_Logic::get_wkc()
{
  return m_wkc;
}

/// EtherCAT Master Application Layer

class EtherCAT_AL
{
  friend class EtherCAT_Router;

public:
  EtherCAT_AL(EtherCAT_DataLinkLayer *m_dll_instance, EC_Logic *m_logic_instance, EtherCAT_PD_Buffer *_pd_buffer_);
  ~EtherCAT_AL();

  /// Get pointer to slaveHandler
  /** @return pointer to the slaveHandler if it exist, or NULL
   otherwise
   @todo As implemented right now, this is an O(N) operation.  Try
   to use some kind of mapping/hash table...
   */
  EtherCAT_SlaveHandler * get_slave_handler(EC_FixedStationAddress station_address);

  /// Get the number of slaves
  /** @return the number of slaves
   */
  unsigned int get_num_slaves()
  {
    return m_num_slaves;
  }

  /// Get an array of pointers to slaves
  /** @return array of pointers to slaves
   */
  EtherCAT_SlaveHandler ** get_slaves()
  {
    return m_slave_handler;
  }

  /// Check the AL is ready (init was successful)
  /** @return true if ready
   */

  bool isReady();

  /// DLL Instance
  EtherCAT_DataLinkLayer * m_dll_instance;

protected:
  /// Init of EtherCAT network.
  /** Counts and identifies the number of slaves in the network, and
   resets the slaves configuration.
   @return true if success
   */
  bool init(void);

  /// Scan the slave network, create the necessary slave Handlers
  /** @return true if success
   @todo if 2 slaves have the same product code and revision, they
   will get the same address!!
   */
  bool scan_slaves(void);

  /// Reset slaves
  /** @return true if success
   */
  bool reset_slaves(void);

  /// Put all slaves in their init state
  /** @return true if success
   */
  bool put_slaves_in_init(void);

private:
  /// Using commonly idx
  EC_Logic * m_logic_instance;

  /// Pointer to the slave Handlers
  EtherCAT_SlaveHandler ** m_slave_handler;

  EtherCAT_PD_Buffer *pd_buffer_;

  /// Get information from the SII (in Autoincrement mode)
  /** @param slave_adp address pointer of the slave to read in
   autoincrement mode
   @param address address of the EEPROM that should be read
   @param a_buffer buffer to put the data in
   @return reading succeeded?
   */
  bool read_SII(uint16_t slave_adp, uint32_t address, unsigned char * a_buffer);

  /// Number of slaves in the setup
  unsigned int m_num_slaves;

  /// Set is init is successful
  bool m_ready;
};

#endif
