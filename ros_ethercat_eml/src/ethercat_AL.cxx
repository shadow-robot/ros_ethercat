// $Id: ethercat_AL.cxx,v 1.21 2006/02/20 15:57:33 kgad Exp $
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

#include "ros_ethercat_eml/ethercat_log.h"
#include "ros_ethercat_eml/ethercat_AL.h"
#include "ros_ethercat_eml/ethercat_slave_conf.h"
#include "ros_ethercat_eml/ethercat_slave_memory.h"
#include "ros_ethercat_eml/ethercat_slave_handler.h"
#include "ros_ethercat_eml/ethercat_dll.h"
#include "ros_ethercat_eml/ethercat_telegram.h"
#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
#include "ros_ethercat_eml/ethercat_logical_addressed_telegram.h"
#include <time.h>

// ==================================================
EC_Logic * EC_Logic::m_instance = NULL;

EC_Logic::EC_Logic()
  :
  m_wkc(0), m_idx(0)
{
}

EC_Logic *
EC_Logic::instance()
{
  if (!m_instance)
  {
    m_instance = new EC_Logic();
  }
  return m_instance;
}

// ==================================================

EtherCAT_AL * EtherCAT_AL::m_instance = NULL;

EtherCAT_AL *
EtherCAT_AL::instance()
{
  if (!m_instance)
  {
    m_instance = new EtherCAT_AL();
  }
  return m_instance;
}

EtherCAT_AL::EtherCAT_AL()
  :
  m_num_slaves(0), m_ready(false)
{
  m_dll_instance = EtherCAT_DataLinkLayer::instance();
  m_slave_db = EtherCAT_SlaveDb::instance();
  m_logic_instance = EC_Logic::instance();

  if (init() == false)
  {
    // Can't use exceptions, since not supported by eCOS f.i.
    ec_log(EC_LOG_FATAL, "EtherCAT_AL:: Can't init network\n");
  }
  m_ready = true;
}

EtherCAT_AL::~EtherCAT_AL()
{
  for (unsigned int i = 0; i < m_num_slaves; i++)
  {
    delete m_slave_handler[i];
  }
  delete[] m_slave_handler;
}

bool
EtherCAT_AL::init(void)
{
  if (scan_slaves())
  {
    if (reset_slaves())
    {
      return put_slaves_in_init();
    }
    else
    {
      ec_log(EC_LOG_FATAL, "Something went wrong while resetting slaves\n");
      return false;
    }
  }
  else
  {
    ec_log(EC_LOG_FATAL, "Something went wrong while scanning network\n");
    return false;
  }
}

bool
EtherCAT_AL::isReady()
{
  return m_ready;
}

bool
EtherCAT_AL::scan_slaves(void)
{
  // Send APRD telegram to count number of slaves
  unsigned char a[1] = {0x00};
  APRD_Telegram counter_tg(m_logic_instance->get_idx(), 0x0000, 0x0000,
                           m_logic_instance->get_wkc(),
                           0x01, a);
  EC_Ethernet_Frame counter_frame(&counter_tg);
  bool succeed = m_dll_instance->txandrx(&counter_frame);
  if (succeed == false)
  {
    ec_log(EC_LOG_FATAL, "Error sending counter frame\n");
    return succeed;
  }
  // Init Number of slaves
  m_num_slaves = counter_tg.get_adp();
  ec_log(EC_LOG_INFO, "EtherCAT AL: Ring contains %d slaves\n", m_num_slaves);
  m_slave_handler = new EtherCAT_SlaveHandler*[m_num_slaves];

  // Initialise Slave Handlers, Reading productcode and revision from SII
  uint16_t adp = 0x0000;
  uint32_t productcode = 0x00000000;
  uint32_t revision = 0x00000000;
  uint32_t serial = 0x00000000;
  const uint16_t SII_datalen = EC_Slave_RD[SII_ControlStatus].size + EC_Slave_RD[SII_Address].size
    + EC_Slave_RD[SII_Data].size;
  unsigned char data[SII_datalen];
  const EtherCAT_SlaveConfig * sconf;
  for (unsigned i = 0; i < SII_datalen; i++)
    data[i] = 0x00;
  for (unsigned int i = 0; i < m_num_slaves; i++)
  {
    for (unsigned j = 0; j < EC_Slave_RD[SII_Data].size; j++)
      data[j] = 0x00;
    succeed = read_SII(adp, EC_ProductCodeAddressInSII, data);
    if (!succeed)
    {
      ec_log(EC_LOG_FATAL, "EC_AL::scan_slaves() Error reading Product code of slave %d\n", i);
      productcode = 0xbaddbadd;
      //return succeed;
    }
    else
    {
      nw2host(data + EC_Slave_RD[SII_ControlStatus].size + EC_Slave_RD[SII_Address].size,
              productcode);
    }
    struct timespec sleept;
    sleept.tv_sec = 0;
    sleept.tv_nsec = 10 * 1000 * 1000; //10ms
    nanosleep(&sleept, 0);

    for (unsigned j = 0; j < EC_Slave_RD[SII_Data].size; j++)
      data[j] = 0x00;
    succeed = read_SII(adp, EC_RevisionAddressInSII, data);
    if (!succeed)
    {
      ec_log(EC_LOG_FATAL, "EC_AL::scan_slaves() Error reading Revision of slave %d\n", i);
      revision = 0xbaddbadd;
      //return succeed;
    }
    else
    {
      nw2host(data + EC_Slave_RD[SII_ControlStatus].size + EC_Slave_RD[SII_Address].size, revision);
    }
    nanosleep(&sleept, 0);

    for (unsigned j = 0; j < EC_Slave_RD[SII_Data].size; j++)
      data[j] = 0x00;
    succeed = read_SII(adp, EC_SerialAddressInSII, data);
    if (!succeed)
    {
      ec_log(EC_LOG_FATAL, "EC_AL::scan_slaves() Error reading Serial of slave %d\n", i);
      serial = 0xbaddbadd;
      //return succeed;
    }
    else
    {
      nw2host(data + EC_Slave_RD[SII_ControlStatus].size + EC_Slave_RD[SII_Address].size, serial);
    }
    nanosleep(&sleept, 0);

    sconf = m_slave_db->find(productcode, revision);
    if (sconf != NULL)
    {
      m_slave_handler[i] = new EtherCAT_SlaveHandler(adp2ringpos(adp), sconf, serial);
      ec_log(EC_LOG_INFO,
             "AL creating SlaveHandler: pos=%d, adr=0x%x, Prod. Code=0x%x, rev=0x%x, Serial=%d\n",
             adp2ringpos(adp),
             (uint16_t) sconf->get_station_address(), productcode, revision, serial);
    }
    else
    { // No such slave found...
      ec_log(EC_LOG_WARNING, "EC_AL Warning: No such slave in db, creating dummy slave\n");
      // Create slave handler
      m_slave_handler[i] = new EtherCAT_SlaveHandler(adp2ringpos(adp), productcode, revision,
                                                     serial, (i + 1), NULL, NULL);
      ec_log(EC_LOG_INFO,
             "AL creating SlaveHandler: pos=%d, Product Code=0x%x, rev=0x%x, Serial=%d\n",
             adp2ringpos(adp),
             productcode, revision, serial);
    }
    // prepare for querying next slave
    adp--;
  }
  return true;
}

bool
EtherCAT_AL::reset_slaves(void)
{
  // Reset FMMUs
  ec_log(EC_LOG_INFO, "AL: resetting FMMUs\n");
  uint16_t ado = EC_Slave_RD[FMMU_0].ado;
  // Whole FMMU area is 0x100...
  static const uint16_t BWR_data_len = 0x100;
  unsigned char BWR_data[BWR_data_len] = {0};
  BWR_Telegram bwr_telegram(m_logic_instance->get_idx(), ado, m_logic_instance->get_wkc(),
                            BWR_data_len, BWR_data);
  EC_Ethernet_Frame bwr_frame(&bwr_telegram);

  bool succeed = m_dll_instance->txandrx(&bwr_frame);
  if (succeed == false)
    return false;

  // 3: Reset Sync Managers
  ec_log(EC_LOG_INFO, "AL: resetting SMs\n");
  // Whole SM area is also 0x100...
  bwr_telegram.set_idx(m_logic_instance->get_idx());
  ado = EC_Slave_RD[Sync_Manager_0].ado;
  bwr_telegram.set_ado(ado);
  bwr_telegram.set_wkc(m_logic_instance->get_wkc());
  return m_dll_instance->txandrx(&bwr_frame);
}

bool
EtherCAT_AL::put_slaves_in_init(void)
{
  ec_log(EC_LOG_INFO, "AL: Setting all slaves in init mode\n");
  // 6: Set device state to init
  EC_ALControl al_control(EC_INIT_STATE, false);
  unsigned char AL_Control_data[EC_Slave_RD[AL_Control].size];
  al_control.dump(AL_Control_data);
  uint16_t ado = EC_Slave_RD[AL_Control].ado;
  uint16_t adp = 0x0000;
  APWR_Telegram AL_control_telegram(m_logic_instance->get_idx(), adp, ado,
                                    m_logic_instance->get_wkc(),
                                    EC_Slave_RD[AL_Control].size,
                                    AL_Control_data);
  EC_Ethernet_Frame AL_control_frame(&AL_control_telegram);

  // 7: Check device state for init
  static const uint16_t AL_Status_Size = EC_Slave_RD[AL_Status].size;
  unsigned char AL_Status_data[AL_Status_Size];
  for (unsigned i = 0; i < AL_Status_Size; ++i)
    AL_Status_data[i] = 0;
  /* Note: cannot initialize data array, since the compiler does not
   recognize AL_Status_Size as being const :-(  An option would be
   to include a for loop, but as this data is filled in by the slave,
   no matter its value, I left it uninitialized for brievety (and
   clarity?).
   */
  ado = EC_Slave_RD[AL_Status].ado;
  APRD_Telegram AL_status_telegram(m_logic_instance->get_idx(), adp, ado,
                                   m_logic_instance->get_wkc(),
                                   AL_Status_Size, AL_Status_data);
  EC_Ethernet_Frame AL_status_frame(&AL_status_telegram);
  uint16_t ringpos = 0;
  bool succeed = true;

  while ((ringpos < m_num_slaves) && (succeed == true))
  {
    succeed = m_dll_instance->txandrx(&AL_control_frame);
    if (succeed == true)
    {
      struct timespec sleept;
      sleept.tv_sec = 0;
      sleept.tv_nsec = 10 * 1000 * 1000; //10ms
      nanosleep(&sleept, 0);

      succeed = m_dll_instance->txandrx(&AL_status_frame);
      if (succeed == true)
      {
        EC_ALStatus status(AL_Status_data);
        if (status.State != EC_INIT_STATE)
        {
          ec_log(EC_LOG_ERROR, "Error: EC slave %d not in init state, AL_status = %x\n", ringpos,
                 status.State);
          succeed = false;
        }
        ringpos++;
        adp = ringpos2adp(ringpos);
        AL_control_telegram.set_adp(adp);
        AL_control_telegram.set_wkc(m_logic_instance->get_wkc());
        AL_control_telegram.set_idx(m_logic_instance->get_idx());

        AL_status_telegram.set_adp(adp);
        AL_status_telegram.set_wkc(m_logic_instance->get_wkc());
        AL_status_telegram.set_idx(m_logic_instance->get_idx());
      }
      else
        ec_log(EC_LOG_ERROR, "EtherCAT_AL: Error sending AL_Status_frame for slave %d\n", ringpos);
    }
    else
    {
      ec_log(EC_LOG_ERROR, "EtherCAT_AL: Error sending AL_Control_frame for slave %d\n", ringpos);
      struct timespec sleept;
      sleept.tv_sec = 0;
      sleept.tv_nsec = 10 * 1000 * 1000; //10ms
      nanosleep(&sleept, 0);
      succeed = true; // Keep trying, for god's sake
    }

  }
  return succeed;
}

// 10 tries was not enough for the optimized version of the code
static const unsigned int EC_SII_MAXTRIES = 100;

bool
EtherCAT_AL::read_SII(uint16_t slave_adp,
                      uint32_t address,
                      unsigned char * a_buffer)
{
  bool succeed;

  // Indicate we want to read from the E2PROM
  const uint16_t SII_control_datalen = EC_Slave_RD[SII_ControlStatus].size
    + EC_Slave_RD[SII_Address].size;
  unsigned char SII_control_data[SII_control_datalen];
  EC_SIIControlStatus siics(false, false, true, false, false, false, false);
  unsigned char * ptr = siics.dump(SII_control_data);
  host2nw(ptr, address);
  APWR_Telegram SII_control_tg(m_logic_instance->get_idx(),
                               slave_adp,
                               EC_Slave_RD[SII_ControlStatus].ado,
                               m_logic_instance->get_wkc(),
                               SII_control_datalen, SII_control_data);
  EC_Ethernet_Frame SII_control_frame(&SII_control_tg);
  succeed = m_dll_instance->txandrx(&SII_control_frame);
  if (!succeed)
  {
    ec_log(EC_LOG_ERROR, "EC_AL::read_SII() Error sending control frame\n");
    return false;
  }
  // BIG FAT WARNING:  USING 2 TELEGRAMS FOR WRITING AS PROGRAMMED
  // AFTER THIS COMMENT DOES NOT WORK,
  // CONTRARY TO WHAT IS WRITTEN IN THE SPEC.  ALSO NOTE THAT THE USED
  // ADDRESSES DO NOT CORRESPOND TO THOSE IN THE SPEC!!
  /*
   const uint16_t SII_control_datalen = EC_Slave_RD[SII_ControlStatus].size;
   unsigned char SII_control_data[SII_control_datalen];
   EC_SIIControlStatus siics(false,false,true,false,false,false,false);
   siics.dump(SII_control_data);
   APWR_Telegram SII_control_tg(m_logic_instance->get_idx(),
   slave_adp,EC_Slave_RD[SII_ControlStatus].ado,
   m_logic_instance->get_wkc(),
   SII_control_datalen,SII_control_data);
   EC_Ethernet_Frame SII_control_frame(&SII_control_tg);
   succeed = m_dll_instance->txandrx(&SII_control_frame);
   if (!succeed) return false;

   // Set the address we want to read from
   const uint16_t SII_address_datalen = EC_Slave_RD[SII_Address].size;
   unsigned char SII_address_data[SII_control_datalen];
   host2nw(SII_address_data,address);
   APWR_Telegram SII_address_tg(m_logic_instance->get_idx(),
   slave_adp,EC_Slave_RD[SII_Address].ado,
   m_logic_instance->get_wkc(),
   SII_address_datalen,SII_address_data);
   EC_Ethernet_Frame SII_address_frame(&SII_address_tg);
   succeed = m_dll_instance->txandrx(&SII_address_frame);
   if (!succeed) return false;
   */

  // The actual read
  const uint16_t SII_datalen = EC_Slave_RD[SII_ControlStatus].size + EC_Slave_RD[SII_Address].size
    + EC_Slave_RD[SII_Data].size;
  APRD_Telegram SII_data_tg(m_logic_instance->get_idx(),
                            slave_adp,
                            EC_Slave_RD[SII_ControlStatus].ado,
                            m_logic_instance->get_wkc(),
                            SII_datalen, a_buffer);
  EC_Ethernet_Frame SII_data_frame(&SII_data_tg);
  unsigned int tries = 0;
  while (tries < EC_SII_MAXTRIES)
  {
    SII_data_tg.set_adp(slave_adp);
    SII_data_tg.set_wkc(m_logic_instance->get_wkc());
    SII_data_tg.set_idx(m_logic_instance->get_idx());
    succeed = m_dll_instance->txandrx(&SII_data_frame);
    if (succeed == true)
    {
      // Check if EEPROM still busy
      EC_SIIControlStatus siics(a_buffer);
      if (siics.Busy)
      {
        ec_log(EC_LOG_WARNING, "EEPROM busy\n");
        struct timespec sleept;
        sleept.tv_sec = 0;
        sleept.tv_nsec = 10 * 1000 * 1000; //10ms
        nanosleep(&sleept, 0);
        tries++;
      }
      else
      {
        if (siics.AcknowledgeError)
        {
          ec_log(EC_LOG_ERROR, "EC_AL::read_SII() Acknowledge error\n");
          return false;
        }
        return succeed;
      }
    }
    tries++;
  }
  ec_log(EC_LOG_ERROR, "EC_AL::read_SII() Max tries exceeded\n");
  return false;
}

EtherCAT_SlaveHandler *
EtherCAT_AL::get_slave_handler(EC_FixedStationAddress station_address)
{
  unsigned int i = 0;
  while (i < m_num_slaves)
  {
    if (m_slave_handler[i]->get_station_address() == station_address)
      return m_slave_handler[i];
    else
      i++;
  }
  ec_log(EC_LOG_WARNING, "EtherCAT_AL: No such slave, returning NULL\n");
  return NULL;
}

