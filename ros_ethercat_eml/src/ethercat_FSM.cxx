// $Id: ethercat_FSM.cxx,v 1.22 2006/02/20 15:57:33 kgad Exp $
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


#include "ros_ethercat_eml/ethercat_FSM.h"
#include "ros_ethercat_eml/ethercat_AL.h"
#include "ros_ethercat_eml/ethercat_slave_handler.h"
#include "ros_ethercat_eml/ethercat_router.h"
#include "ros_ethercat_eml/ethercat_process_data.h"
#include "ros_ethercat_eml/ethercat_slave_memory.h"
#include "ros_ethercat_eml/ethercat_dll.h"
#include "ros_ethercat_eml/ethercat_telegram.h"
#include "ros_ethercat_eml/ethercat_frame.h"
#include "ros_ethercat_eml/ethercat_device_addressed_telegram.h"
#include <time.h>

EC_ESM_InitState EC_ESM_State::initState;
EC_ESM_PreOpState EC_ESM_State::preopState;
EC_ESM_SafeOpState EC_ESM_State::safeopState;
EC_ESM_OpState EC_ESM_State::opState;

EC_ESM_Ops::EC_ESM_Ops(EtherCAT_SlaveHandler * a_SH,
                       EtherCAT_DataLinkLayer *_m_dll_instance,
                       EC_Logic *_m_logic_instance,
                       EtherCAT_PD_Buffer *_m_pdbuf_instance) :
  m_logic_instance(_m_logic_instance),
  m_router_instance(NULL),
  m_dll_instance(_m_dll_instance),
  m_SH(a_SH),
  m_pdbuf_instance(_m_pdbuf_instance)
{
}

void EC_ESM_Ops::setRouter(EtherCAT_Router* _router)
{
  m_router_instance = _router;
}

// Maximum number of tries setting the state
static const unsigned int EC_ESM_OPS_MAX_RETRIES = 10;

bool EC_ESM_Ops::set_state(EC_State a_state)
{
  static const uint16_t AL_Control_Size = EC_Slave_RD[AL_Control].size;
  EC_ALControl al_control(a_state, false);
  unsigned char AL_Control_data[AL_Control_Size];
  al_control.dump(AL_Control_data);
  NPWR_Telegram AL_control_telegram(m_logic_instance->get_idx(),
                                    m_SH->get_station_address(),
                                    EC_Slave_RD[AL_Control].ado,
                                    m_logic_instance->get_wkc(),
                                    AL_Control_Size,
                                    AL_Control_data);
  EC_Ethernet_Frame AL_control_frame(&AL_control_telegram);

  bool succeed;
  unsigned int tries = 0;

  while (tries < EC_ESM_OPS_MAX_RETRIES)
  {
    succeed = m_dll_instance->txandrx(&AL_control_frame);
    if (succeed == true)
    {
      static const uint16_t AL_Status_Size = EC_Slave_RD[AL_Status].size;
      unsigned char AL_Status_data[AL_Status_Size];
      for (unsigned i = 0; i < AL_Status_Size; ++i)
        AL_Status_data[i] = 0;
      NPRD_Telegram AL_status_telegram(m_logic_instance->get_idx(),
                                       m_SH->get_station_address(),
                                       EC_Slave_RD[AL_Status].ado,
                                       m_logic_instance->get_wkc(), AL_Status_Size, AL_Status_data);
      EC_Ethernet_Frame AL_status_frame(&AL_status_telegram);
      struct timespec sleept;
      sleept.tv_sec = 0;
      sleept.tv_nsec = 10 * 1000 * 1000; //10ms
      nanosleep(&sleept, 0);
      succeed = m_dll_instance->txandrx(&AL_status_frame);
      if (succeed == true)
      {
        EC_ALStatus status(AL_Status_data);
        if ((status.State == a_state) && (status.Change == false))
          return true;
        else
          ec_log(EC_LOG_WARNING, "EC_ESM_Ops::set_state() Warning: State trans. failed (try %d), desired=%x, status=%x\n", tries, a_state, status.State);
      }
    }
    else
    {
      ec_log(EC_LOG_WARNING, "EC_ESM_Ops::set_state() Warning: Error sending control frame (try %d)\n", tries);
      struct timespec sleept;
      sleept.tv_sec = 0;
      sleept.tv_nsec = 10 * 1000 * 1000; //10ms
      nanosleep(&sleept, 0);
    }

    AL_control_telegram.set_idx(m_logic_instance->get_idx());
    AL_control_telegram.set_wkc(m_logic_instance->get_wkc());
    al_control.dump(AL_Control_data);
    tries++;
  }
  // Should never get here...
  ec_log(EC_LOG_ERROR, "EC_ESM_Ops::set_state() failed to set state after %d tries", EC_ESM_OPS_MAX_RETRIES);
  return false;
}

bool EC_ESM_Ops::start_mbx_comm()
{
  // Checking the current state of FSM is probably not necessary, since
  // this function can only be called from the appropriate state (at
  // least in theory :-)

  // FIXME Check PDI.  Or is this slave specific and should this be
  // passed in via slave specific init commands? Configure all other
  // DLL registers here...

  // Set station address
  static const uint16_t address_datalen = EC_Slave_RD[ECAT_Station_Address].size;
  unsigned char address_data[address_datalen];
  (m_SH->get_station_address()).dump(address_data);
  APWR_Telegram address_tg(m_logic_instance->get_idx(),
                           ringpos2adp(m_SH->get_ring_position()),
                           EC_Slave_RD[ECAT_Station_Address].ado,
                           (m_logic_instance->get_wkc()),
                           address_datalen,
                           address_data);
  EC_Ethernet_Frame address_frame(&address_tg);
  bool succeed = m_dll_instance->txandrx(&address_frame);
  if (succeed == false)
  {
    ec_log(EC_LOG_ERROR, "Error setting Fixed Station Address\n");
    return succeed;
  }
  struct timespec sleept;
  sleept.tv_sec = 0;
  sleept.tv_nsec = 10 * 1000 * 1000; //10ms
  nanosleep(&sleept, 0);

  // From here on, we can use NP telegrams

  // MBX initialisation for complex slaves
  if (m_SH->is_complex())
  {
    // Write MBX configuration to slave
    unsigned char mbx_conf_data[EC_Slave_RD[Sync_Manager_0].size];
    (m_SH->get_mbx_config()->SM0).dump(mbx_conf_data);
    NPWR_Telegram mbx_conf_tg(m_logic_instance->get_idx(),
                              m_SH->get_station_address(),
                              EC_Slave_RD[Sync_Manager_0].ado,
                              m_logic_instance->get_wkc(),
                              EC_Slave_RD[Sync_Manager_0].size,
                              mbx_conf_data);
    EC_Ethernet_Frame mbx_conf_frame(&mbx_conf_tg);
    succeed = m_dll_instance->txandrx(&mbx_conf_frame);
    if (succeed == false)
    {
      ec_log(EC_LOG_ERROR, "Error setting SM0 conf for mbx\n");
      return succeed;
    }
    nanosleep(&sleept, 0);

    // Do the same for the second mbx
    (m_SH->get_mbx_config()->SM1).dump(mbx_conf_data);
    mbx_conf_tg.set_idx(m_logic_instance->get_idx());
    mbx_conf_tg.set_ado(EC_Slave_RD[Sync_Manager_1].ado);
    mbx_conf_tg.set_wkc(m_logic_instance->get_wkc());
    succeed = m_dll_instance->txandrx(&mbx_conf_frame);
    if (succeed == false)
    {
      ec_log(EC_LOG_ERROR, "Error setting SM1 conf for mbx\n");
      return succeed;
    }
    nanosleep(&sleept, 0);

    // Start mbx communication via router...
    m_router_instance->start();
  }

  // Set state to PREOP
  return set_state(EC_PREOP_STATE);
}

bool EC_ESM_Ops::stop_mbx_comm()
{
  return set_state(EC_INIT_STATE);

  if (m_SH->is_complex())
  {
    // Stop mbx communication via router...
    m_router_instance->stop();
  }
  return true;
}

bool EC_ESM_Ops::start_input_update()
{
  bool succeed = true;

  // Write FMMU channels
  static const uint16_t fmmu_data_len = EC_Slave_RD[FMMU_0].size;
  unsigned char fmmu_data[fmmu_data_len];
  uint16_t adp = m_SH->get_station_address();
  uint16_t ado = 0x0000;

  NPWR_Telegram fmmu_tg(m_logic_instance->get_idx(), adp, ado, (m_logic_instance->get_wkc()), fmmu_data_len, fmmu_data);
  EC_Ethernet_Frame fmmu_frame(&fmmu_tg);
  unsigned int i = 0;

  assert(m_SH->get_fmmu_config() != NULL);
  while ((succeed == true) && (i < m_SH->get_fmmu_config()->get_num_used_fmmus()))
  {
    (*(m_SH->get_fmmu_config()))[i].dump(fmmu_data);
    ado = EC_Slave_RD[FMMUx(i)].ado;
    fmmu_tg.set_ado(ado);
    succeed = m_dll_instance->txandrx(&fmmu_frame);

    // increase idx and reset wkc
    fmmu_tg.set_idx(m_logic_instance->get_idx());
    fmmu_tg.set_wkc((m_logic_instance->get_wkc()));
    i++;
  }

  if (succeed == false)
  {
    ec_log(EC_LOG_ERROR, "error writing fmmu config\n");
    return false;
  }

  unsigned int j;
  // Part two: write Sync Managers
  if (m_SH->is_complex() == true) // write sync managers starting from 2
    j = 2;
  else // write sync managers starting from 0
    j = 0;

  static const uint16_t sm_data_len = EC_Slave_RD[Sync_Manager_0].size;
  unsigned char sm_data[sm_data_len];
  NPWR_Telegram sm_tg(m_logic_instance->get_idx(), adp, ado, (m_logic_instance->get_wkc()), sm_data_len, sm_data);
  EC_Ethernet_Frame sm_frame(&sm_tg);
  i = 0;
  while ((succeed == true) && (i < m_SH->get_pd_config()->get_num_used_sms()))
  {
    (*(m_SH->get_pd_config()))[i].dump(sm_data);
    ado = EC_Slave_RD[Sync_Managerx(j)].ado;
    sm_tg.set_ado(ado);
    succeed = m_dll_instance->txandrx(&sm_frame);

    // increase idx and reset wkc
    sm_tg.set_idx(m_logic_instance->get_idx());
    sm_tg.set_wkc((m_logic_instance->get_wkc()));
    i++;
    j++;
  }
  if (succeed == false)
  {
    ec_log(EC_LOG_ERROR, "error writing SM config\n");
    return false;
  }

  // Note:  In case of complex slaves, check if SDO-Download services
  // to be sent to slave and send them if necessary...

  succeed = set_state(EC_SAFEOP_STATE);
  if (succeed)
    m_pdbuf_instance->start();
  // Ask for start PD transmission in master
  return succeed;

}

bool EC_ESM_Ops::stop_input_update()
{
  // Ask for stopping PD transmission in master
  m_pdbuf_instance->stop();
  return set_state(EC_PREOP_STATE);
}

bool EC_ESM_Ops::start_output_update()
{
  /* According to the spec (9.1.3.5 start output update (master)), make sure to
     transmit "valid" output data to the slaves.  However, AFAIS, the
     master cannot tell whether output data are valid or not.  That is
     up to the slaves.  Moreover, in (9.1.2.5 start output update
     (slave)), the slave has to receive the al control indication
     first and then starts writing valid output data , so this is contrary to
     what is told in the master.
   */
  return set_state(EC_OP_STATE);
}

bool EC_ESM_Ops::stop_output_update()
{
  return set_state(EC_SAFEOP_STATE);
}
// ==================================================

EC_ESM::EC_ESM(EtherCAT_SlaveHandler * a_SH,
               EtherCAT_DataLinkLayer *_m_dll_instance,
               EC_Logic *_m_logic_instance,
               EtherCAT_PD_Buffer *_m_pdbuf_instance)
  : EC_ESM_Ops(a_SH,
               _m_dll_instance,
               _m_logic_instance,
               _m_pdbuf_instance)
{
  m_esm_state = &EC_ESM_State::initState;
}

// ==================================================

EC_State EC_ESM_InitState::get_state() const
{
  return EC_INIT_STATE;
}

bool EC_ESM_InitState::to_state(EC_ESM * a_ESM, EC_State a_state)
{
  bool succeed;
  switch (a_state)
  {
    case EC_INIT_STATE:
      succeed = true;
      break;
    case EC_PREOP_STATE:
      succeed = a_ESM->start_mbx_comm();
      if (succeed) a_ESM->setState(&preopState);
      break;
    case EC_SAFEOP_STATE:
      succeed = a_ESM->start_mbx_comm();
      if (succeed)
      {
        a_ESM->setState(&preopState);
        succeed = a_ESM->to_state(EC_SAFEOP_STATE);
      }
      break;
    case EC_OP_STATE:
      succeed = a_ESM->start_mbx_comm();
      if (succeed)
      {
        a_ESM->setState(&preopState);
        succeed = a_ESM->to_state(EC_OP_STATE);
      }
      break;
    default:
      succeed = false;
      break;
      // FIXME implement bootstrap state...
  }
  return succeed;
}

EC_State EC_ESM_PreOpState::get_state() const
{
  return EC_PREOP_STATE;
}

bool EC_ESM_PreOpState::to_state(EC_ESM * a_ESM, EC_State a_state)
{
  bool succeed;
  switch (a_state)
  {
    case EC_INIT_STATE:
      succeed = a_ESM->stop_mbx_comm();
      if (succeed) a_ESM->setState(&initState);
      break;
    case EC_PREOP_STATE:
      succeed = true;
      break;
    case EC_SAFEOP_STATE:
      succeed = a_ESM->start_input_update();
      if (succeed) a_ESM->setState(&safeopState);
      break;
    case EC_OP_STATE:
      succeed = a_ESM->start_input_update();
      if (succeed)
      {
        a_ESM->setState(&safeopState);
        succeed = a_ESM->to_state(EC_OP_STATE);
      }
      break;
    default:
      succeed = false;
      break;
      // FIXME implement bootstrap state...
  }
  return succeed;
}

EC_State EC_ESM_SafeOpState::get_state() const
{
  return EC_SAFEOP_STATE;
}

bool EC_ESM_SafeOpState::to_state(EC_ESM * a_ESM, EC_State a_state)
{
  bool succeed;
  switch (a_state)
  {
    case EC_INIT_STATE:
      succeed = a_ESM->stop_input_update();
      if (succeed)
      {
        succeed = a_ESM->stop_mbx_comm();
        if (succeed) a_ESM->setState(&initState);
      }
      break;
    case EC_PREOP_STATE:
      succeed = a_ESM->stop_input_update();
      if (succeed) a_ESM->setState(&preopState);
      break;
    case EC_SAFEOP_STATE:
      succeed = true;
      break;
    case EC_OP_STATE:
      succeed = a_ESM->start_output_update();
      if (succeed) a_ESM->setState(&opState);
      break;
    default:
      succeed = false;
      break;
      // FIXME implement bootstrap state...
  }
  return succeed;
}

EC_State EC_ESM_OpState::get_state() const
{
  return EC_OP_STATE;
}

bool EC_ESM_OpState::to_state(EC_ESM * a_ESM, EC_State a_state)
{
  bool succeed;
  switch (a_state)
  {
    case EC_INIT_STATE:
      succeed = a_ESM->stop_output_update();
      if (succeed)
      {
        succeed = a_ESM->stop_input_update();
        if (succeed)
        {
          succeed = a_ESM->stop_mbx_comm();
          if (succeed) a_ESM->setState(&initState);
        }
      }
      break;
    case EC_PREOP_STATE:
      succeed = a_ESM->stop_output_update();
      if (succeed)
      {
        succeed = a_ESM->stop_input_update();
        if (succeed) a_ESM->setState(&preopState);
      }
      break;
    case EC_SAFEOP_STATE:
      succeed = a_ESM->stop_output_update();
      if (succeed) a_ESM->setState(&safeopState);
      break;
    case EC_OP_STATE:
      succeed = true;
      break;
    default:
      succeed = false;
      break;
      // FIXME implement bootstrap state...
  }
  return succeed;
}
