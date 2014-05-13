// $Id: ethercat_slave_conf.h,v 1.14 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_slave_conf__
#define __ethercat_slave_conf__

#include "ethercat/ethercat_defs.h"
#include "dll/ethercat_slave_memory.h"

/// FMMU Configuration of a slave
class EtherCAT_FMMU_Config
{
 public:
  /// Constructor
  /** @param a_num_used_fmmus the number of used FMMUs for this slave
      (configuration)
   */
  EtherCAT_FMMU_Config(unsigned int a_num_used_fmmus);
  virtual ~EtherCAT_FMMU_Config();

  /// Returns the number of used FMMUs
  /** @return the number of used FMMUs
   */
  unsigned int get_num_used_fmmus() const {return m_num_used_fmmus;}

  /// Operator []
  /** @param i which FMMU
      @return ref to the FMMU
  */
  EC_FMMU & operator[](unsigned int i);
  /// Operator [] (const version)
  /** @param i which FMMU
      @return ref to the FMMU
  */
  const EC_FMMU & operator[](unsigned int i) const;

 protected:
  EC_FMMU * fmmus;
  unsigned int m_num_used_fmmus;
};

/// Process Data Configuration
/** This class contains the configuration of the Sync Managers used
    for the process data transmission
*/
class EtherCAT_PD_Config
{
 public:
  /// Constructor
  /** @param a_num_used_sms The number of used Sync Managers for
      transmitting process data
  */
  EtherCAT_PD_Config(unsigned int a_num_used_sms);
  virtual ~EtherCAT_PD_Config();

  /// Retrieve the number of used Sync Managers
  /** @return the number of used Sync Managers of this config
   */
  unsigned int get_num_used_sms() const {return m_num_used_sms;}

  /// Get a sync Manager configuration
  /** @param i which Sync Manager 
      @return ref to the Sync Manager
  */
  EC_SyncMan & operator[](unsigned int i);
  /// Get a sync Manager configuration (const version)
  /** @param i which Sync Manager 
      @return ref to the Sync Manager
  */
  const EC_SyncMan & operator[](unsigned int i) const;

 protected:
  /// Pointer to the Sync Manager array
  EC_SyncMan * sms;
  /// Number of used Sync Managers
  unsigned int m_num_used_sms;
};

/// Mailbox Configuration (Sync Manager 0 and 1)
typedef struct
{
  /// Sync Manager 0: For Master to slave communication
  EC_SyncMan SM0;
  /// Sync Manager 1: For Slave to Master communication
  EC_SyncMan SM1;
} EtherCAT_MbxConfig;

/// Configuration of EtherCAT Slave
/** @todo Build this class by parsing (xml-)configuration file
 */
class EtherCAT_SlaveConfig
{
 public:
  /// Constructor
  /** @param a_product_code product code of the slave
      @param a_revision revision of the slave
      @param a_station_address address of the station as given in
      config file
      @param a_fmmu_config pointer to configuration of fmmus as
      created when parsing config file
      @param a_pd_config pointer to configuration of SM for process
      data
      @param a_mbx_config MBX configuration if this slave is complex.
      Default argument is NULL for simple slaves
      @note  CanOpen stuff for complex slaves such as the PDO Mapping, the
      Sync Manager PDO assign objects and the startup objects are not
      implemented (yet)
      See p. 106 and 162 of spec.
      // EtherCAT_PDO_Mapping m_pdo_mapping;
      // EtherCAT_SM_PDO_Assign m_sm_pdo_assign;
      // EtherCAT_Startup_Objects m_startup_objects;
  */
  EtherCAT_SlaveConfig(EC_UDINT a_product_code,
		       EC_UDINT a_revision,
		       EC_FixedStationAddress a_station_address,
		       EtherCAT_FMMU_Config * a_fmmu_config,
		       EtherCAT_PD_Config * a_pd_config,
		       EtherCAT_MbxConfig * a_mbx_config = NULL);
  
  virtual ~EtherCAT_SlaveConfig();

 public:
  /// Is this a complex slave?
  bool is_complex(void) const { return m_complex; };
  /// Get product code
  EC_UDINT get_product_code() const { return m_product_code; };
  /// Get revision
  EC_UDINT get_revision() const { return m_revision; };
  /// Get station address
  EC_FixedStationAddress get_station_address() const {return m_station_address; };
  /// Get FMMU config to be written when going to preop
  const EtherCAT_FMMU_Config * get_fmmu_config() const {return m_fmmu_config; };
  void set_fmmu_config(EtherCAT_FMMU_Config *new_config) { m_fmmu_config = new_config; };
  /// Get PD Configuration (SMS)
  const EtherCAT_PD_Config * get_pd_config() const {return m_pd_config; };
  void set_pd_config(EtherCAT_PD_Config *new_config) { m_pd_config = new_config; };
  /// Get configuration of Sync Man channel 0 and 1 for MBX
  /** @return pointer to config in case of complex slave, or NULL for
      a simple slave
  */
  const EtherCAT_MbxConfig * get_mbx_config() const { return m_mbx_config; };
  void set_mbx_config(EtherCAT_MbxConfig *new_config);
  /// Check if the SlaveConfig is already used
  bool is_used() { return used; };
  /// The SlaveConfig is now used
  void set_used() { used = true; };

 protected:
  /// Product code
  EC_UDINT m_product_code;
  /// Revision
  EC_UDINT m_revision;
  /// Station address
  EC_FixedStationAddress m_station_address;
  /// FMMU config to be written when going to preop
  EtherCAT_FMMU_Config * m_fmmu_config;
  /// PD Configuration (SMS)
  EtherCAT_PD_Config * m_pd_config;  /// Position in the EtherCAT logical Ring
  /// Configuration of Sync Man channel 0 and 1 for MBX
  EtherCAT_MbxConfig * m_mbx_config;

  bool m_complex;
  
  bool used;

};

/// Database of EtherCAT slave configurations
/** @todo allow dynamic updating by adding of removing a slave
    configuration
*/
class EtherCAT_SlaveDb
{
 public:
  /// Singleton
  static EtherCAT_SlaveDb * instance(unsigned int num_slaves = 0);
  
  virtual ~EtherCAT_SlaveDb();

  EtherCAT_SlaveConfig * operator[](unsigned int i);
  const EtherCAT_SlaveConfig * operator[](unsigned int i) const;
  void set_conf(EtherCAT_SlaveConfig * conf, unsigned int i);
  
  /// Find a configuration...
  /** @param productcode product code
      @param revision revision
      @return Pointer to slave configuration if found, or NULL
  otherwise)
  */
  const EtherCAT_SlaveConfig * find(EC_UDINT productcode,
				    EC_UDINT revision) const;
 protected:
  /// Constructor
  EtherCAT_SlaveDb(unsigned int num_slaves);

 private:
  static EtherCAT_SlaveDb * m_instance;
  
  EtherCAT_SlaveConfig ** m_sc;
  unsigned int m_num_slaves;

};

#endif // __ethercat_slave_conf__
