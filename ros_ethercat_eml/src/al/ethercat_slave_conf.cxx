// $Id: ethercat_slave_conf.cxx,v 1.9 2006/02/20 15:57:33 kgad Exp $
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

 
#include "ros_ethercat_eml/al/ethercat_slave_conf.h"

#include "ros_ethercat_eml/dll/ethercat_slave_memory.h"
#include "ros_ethercat_eml/dll/ethercat_dll.h"
#include "ros_ethercat_eml/dll/ethercat_frame.h"
#include "ros_ethercat_eml/dll/ethercat_device_addressed_telegram.h"
#include "ros_ethercat_eml/al/ethercat_master.h"

EtherCAT_FMMU_Config::EtherCAT_FMMU_Config(unsigned int a_num_used_fmmus)
  : m_num_used_fmmus(a_num_used_fmmus)
{
  fmmus = new EC_FMMU[m_num_used_fmmus];
}

EtherCAT_FMMU_Config::~EtherCAT_FMMU_Config()
{
  delete[] fmmus;
}

EC_FMMU &
EtherCAT_FMMU_Config::operator[](unsigned int i)
{
  assert(i < m_num_used_fmmus);
  return fmmus[i];
}

const EC_FMMU &
EtherCAT_FMMU_Config::operator[](unsigned int i) const
{
  assert(i < m_num_used_fmmus);
  return fmmus[i];
}

// ==================================================

EtherCAT_PD_Config::EtherCAT_PD_Config(unsigned int a_num_used_sms)
  : m_num_used_sms(a_num_used_sms)
{
  sms = new EC_SyncMan[m_num_used_sms];
}

EtherCAT_PD_Config::~EtherCAT_PD_Config()
{
  delete[] sms;
}

EC_SyncMan &
EtherCAT_PD_Config::operator[](unsigned int i)
{
  assert(i < m_num_used_sms);
  return sms[i];
}

const EC_SyncMan &
EtherCAT_PD_Config::operator[](unsigned int i) const
{
  assert(i < m_num_used_sms);
  return sms[i];
}

// ==================================================

EtherCAT_SlaveConfig::EtherCAT_SlaveConfig(uint32_t a_product_code,
					 uint32_t a_revision,
					 EC_FixedStationAddress a_station_address,
					 EtherCAT_FMMU_Config * a_fmmu_config,
					 EtherCAT_PD_Config * a_pd_config,
					 EtherCAT_MbxConfig * a_mbx_config)
  : m_product_code(a_product_code), m_revision(a_revision), 
    m_station_address(a_station_address), m_fmmu_config(a_fmmu_config),
    m_pd_config(a_pd_config), m_mbx_config(a_mbx_config), used(false)
{
  if (m_mbx_config == NULL) 
    m_complex = false;
  else
    m_complex = true;
}

void EtherCAT_SlaveConfig::set_mbx_config(EtherCAT_MbxConfig *new_config)
{ 
  m_mbx_config = new_config;
  if (m_mbx_config == NULL) 
    m_complex = false;
  else
    m_complex = true; 
}

EtherCAT_SlaveConfig::~EtherCAT_SlaveConfig(){}

// ==================================================
EtherCAT_SlaveDb * EtherCAT_SlaveDb::m_instance = NULL;

EtherCAT_SlaveDb *
EtherCAT_SlaveDb::instance(unsigned int num_slaves)
{
  if (!m_instance){
    m_instance = new EtherCAT_SlaveDb(num_slaves);
  }
  return m_instance;
}

EtherCAT_SlaveDb::EtherCAT_SlaveDb(unsigned int num_slaves)
  : m_num_slaves(num_slaves)
{
  m_sc = new EtherCAT_SlaveConfig*[num_slaves];
}

EtherCAT_SlaveDb::~EtherCAT_SlaveDb()
{
  delete[] m_sc;
}

EtherCAT_SlaveConfig * 
EtherCAT_SlaveDb::operator[](unsigned int i)
{
  assert(i < m_num_slaves);
  return m_sc[i];
}

const EtherCAT_SlaveConfig * 
EtherCAT_SlaveDb::operator[](unsigned int i) const
{
  assert(i < m_num_slaves);
  return m_sc[i];
}

void
EtherCAT_SlaveDb::set_conf(EtherCAT_SlaveConfig * conf, 
			    unsigned int i)
{
  assert(i < m_num_slaves);
  m_sc[i] = conf;
}

const EtherCAT_SlaveConfig * 
EtherCAT_SlaveDb::find(uint32_t productcode,
		       uint32_t revision) const
{
  unsigned int i = 0;
  while(i < m_num_slaves){
	  if ( !m_sc[i]->is_used() && (m_sc[i]->get_product_code() == productcode) &&
			(m_sc[i]->get_revision() == revision) ) {
		  m_sc[i]->set_used();
		  return m_sc[i];
	  }
    else i++;
  }
  ec_log(EC_LOG_WARNING, "EtherCAT_SlaveDb: No such Config, returning NULL!\n");
  return NULL;
}





