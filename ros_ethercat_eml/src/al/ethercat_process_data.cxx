// $Id: ethercat_process_data.cxx,v 1.10 2006/02/20 15:57:33 kgad Exp $
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

 
#include "al/ethercat_process_data.h"
#include "al/ethercat_slave_handler.h"
#include "al/ethercat_AL.h"
#include "al/ethercat_master.h"
#include "dll/ethercat_dll.h"

EtherCAT_PD_Buffer * EtherCAT_PD_Buffer::m_instance = 0;

EtherCAT_PD_Buffer *
EtherCAT_PD_Buffer::instance()
{
  if (!m_instance) {
    m_instance = new EtherCAT_PD_Buffer();
  }
  return m_instance;
}

// al_instance cannot be initiated right now, since this results in a
// circular instantiation al_instance calls router_instance etc.
EtherCAT_PD_Buffer::EtherCAT_PD_Buffer()
  : m_is_running(0)
{
  for (unsigned i=0;i<MAX_CHUNKS;++i) {
    m_lrw_telegram[i] = new LRW_Telegram(0x00,0x00010000,0x00,0,NULL);
    m_lrw_frame[i] = new EC_Ethernet_Frame(m_lrw_telegram[i]);
  }

  // get pointer to DLL and logic
  m_dll_instance = EtherCAT_DataLinkLayer::instance();
  m_logic_instance = EC_Logic::instance();
}

EtherCAT_PD_Buffer::~EtherCAT_PD_Buffer()
{
  for (unsigned i=0;i<MAX_CHUNKS;++i) {
    delete m_lrw_telegram[i];
    m_lrw_telegram[i] = NULL;
    delete m_lrw_frame[i];
    m_lrw_frame[i];
  }
}

void 
EtherCAT_PD_Buffer::start()
{
  ++m_is_running;
}

void 
EtherCAT_PD_Buffer::stop()
{ 
  if (m_is_running > 0)
    --m_is_running;
  else
    ec_log(EC_LOG_INFO, "EtherCAT_PD_Buffer already stopped...\n");
}

bool
EtherCAT_PD_Buffer::txandrx(size_t datalen, unsigned char * data)
{
  //define MAX_CHUNKS 4
  //define CHUNK_SIZE 1486

  if (datalen>(CHUNK_SIZE*MAX_CHUNKS)) {
    ec_log(EC_LOG_ERROR, "PD_Buffer: Too much data (%zd) to send in %d chunks of %d bytes\n", 
           datalen, MAX_CHUNKS, CHUNK_SIZE);
    return false;
  }

  int handles[MAX_CHUNKS];
  for (unsigned i=0;i<MAX_CHUNKS;++i)
    handles[i]=-1;

  bool success = true;
  if ( m_is_running != 0)
    // In case only starting when all slaves in the appropriate state
    // this becomes something like
    // if ( EtherCAT_PD_Buffer::m_al_instance->get_num_slaves() == m_running)
    {
      int dst = 0x00010000;
      unsigned index=0;
      while (datalen > 0)
      {
        assert(index<MAX_CHUNKS);
        size_t chunk_size = datalen <  CHUNK_SIZE ? datalen : CHUNK_SIZE;
        LRW_Telegram* a_lrw_telegram(m_lrw_telegram[index]);
        a_lrw_telegram->set_idx(m_logic_instance->get_idx());
        a_lrw_telegram->set_wkc(m_logic_instance->get_wkc());
        a_lrw_telegram->set_datalen(chunk_size);
        a_lrw_telegram->set_data(data);
        a_lrw_telegram->set_adr(dst);

        int result = m_dll_instance->tx(m_lrw_frame[index]);
        if (result < 0)
        {
          ec_log(EC_LOG_ERROR, "PD_Buffer: Error sending PD\n");
          success = false;
          break;
        }
        handles[index] = result;
        datalen -= chunk_size;
        data += chunk_size;
        dst += chunk_size;
        ++index;
      }

      // Receive every packet that was sent  
      for(int i=index-1; i>=0; --i) {
        if (handles[i] != -1) {
          if (!m_dll_instance->rx(m_lrw_frame[i],handles[i])) {
            ec_log(EC_LOG_ERROR, "PD_Buffer: Error receiving PD\n");
            success = false;
          }
        }
      }
    }
  
  return success;
}


	  
      



