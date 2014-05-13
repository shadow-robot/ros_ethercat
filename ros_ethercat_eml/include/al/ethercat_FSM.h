// $Id: ethercat_FSM.h,v 1.16 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_fsm__
#define __ethercat_fsm__

/* Note, this class is supposed to be programmed according to the
   FSM design Pattern.
*/

// forward declarations
class EtherCAT_AL;
class EC_Logic;
class EtherCAT_SlaveHandler;
class EtherCAT_DataLinkLayer;
class EtherCAT_Router;
class EtherCAT_PD_Buffer;

// FIXME how to do forward declaration of typedef enum?, with "extern"
// keyword?
#include "dll/ethercat_slave_memory.h"

/// EtherCAT State Machine Operations
class EC_ESM_Ops 
{
 protected:
  /// Start MBX communication
  bool start_mbx_comm();
  /// Stop MBX communication
  bool stop_mbx_comm();
  /// Start Input update
  bool start_input_update();
  /// Stop Input update
  bool stop_input_update();
  /// Start Output update
  bool start_output_update();
  /// Stop Output update
  bool stop_output_update();

  virtual ~EC_ESM_Ops(){};
 protected:
  /// Constructor
  /** @param a_SH pointer to slave handler
   */
  EC_ESM_Ops(EtherCAT_SlaveHandler * a_SH);
  
  EtherCAT_DataLinkLayer * m_dll_instance;
  EC_Logic * m_logic_instance;
  EtherCAT_SlaveHandler * m_SH;
  EtherCAT_Router * m_router_instance;
  EtherCAT_PD_Buffer * m_pdbuf_instance;

  /// Change state of Slave
  /** @return true if succeeded
      @pre The station address of the slave is set (this function uses
      npwr telegrams!
  */
  bool set_state(EC_State a_state);
};

// More forward declarations :-)
class EC_ESM;
class EC_ESM_InitState;
class EC_ESM_PreOpState;
class EC_ESM_SafeOpState;
class EC_ESM_OpState;

/// ESM State Interface class
/** @todo The bootstrap state is currently not implemented
 */
class EC_ESM_State
{
  friend class EC_ESM;
  
 public:
  virtual ~EC_ESM_State(){};
  /// Change the state of an EtherCAT state machine
  /** @param a_ESM pointer to the EtherCAT state machine
      @param a_state to which state?  Possibilities are
      - EC_INIT_STATE = 0x01,
      - EC_PREOP_STATE = 0x02,
      - EC_BOOTSTRAP_STATE = 0x03,
      - EC_SAFEOP_STATE = 0x04,
      - EC_OP_STATE = 0x08
  */
  virtual bool to_state(EC_ESM * a_ESM, EC_State a_state) = 0;  
  virtual EC_State get_state( ) const = 0;
 protected:
  static EC_ESM_InitState initState;
  static EC_ESM_PreOpState preopState;
  static EC_ESM_SafeOpState safeopState;
  static EC_ESM_OpState opState;
};

/// EtherCAT State Machine
/** @todo Unexpected transitions in the state of the slave (via slaves
    application, only for complex slaves) are not dealt with for
    now...  This should probably be fixed using a special mailbox msg or a polling
    mechanism but AFAIS this is undocumented in the spec (see
    8.2.4.4.6 Stop output update)...
*/
class EC_ESM : public EC_ESM_Ops
{
  friend class EC_ESM_State;
  friend class EC_ESM_InitState;
  friend class EC_ESM_PreOpState;
  friend class EC_ESM_SafeOpState;
  friend class EC_ESM_OpState;

 public:
  /// State Transitions
  /** @param a_state state to go to. Possibilities are
      - EC_INIT_STATE = 0x01,
      - EC_PREOP_STATE = 0x02,
      - EC_BOOTSTRAP_STATE = 0x03,
      - EC_SAFEOP_STATE = 0x04,
      - EC_OP_STATE = 0x08
  */
  bool to_state(EC_State a_state){ return (m_esm_state->to_state(this,a_state)); }
  
  EC_State get_state() { return m_esm_state->get_state(); } 

  virtual ~EC_ESM(){};
  
 protected:
  /// Set the internal state
  void setState(EC_ESM_State * a_esm_state){ m_esm_state = a_esm_state; }

  /// Constructor
  /** @param a_SH pointer to Slave Handler this FSM concerns
   */
  EC_ESM(EtherCAT_SlaveHandler * a_SH);

 private:
  EC_ESM_State * m_esm_state;
};

class EC_ESM_InitState : public EC_ESM_State
{
 public:
  virtual EC_State get_state( ) const;
  virtual bool to_state(EC_ESM * a_ESM, EC_State a_state);
};

class EC_ESM_PreOpState : public EC_ESM_State
{
 public:
  virtual EC_State get_state( ) const;
  virtual bool to_state(EC_ESM * a_ESM, EC_State a_state);
};

class EC_ESM_SafeOpState : public EC_ESM_State
{
 public:
  virtual EC_State get_state( ) const;
  virtual bool to_state(EC_ESM * a_ESM, EC_State a_state);
};

class EC_ESM_OpState : public EC_ESM_State
{
 public:
  virtual EC_State get_state( ) const;
  virtual bool to_state(EC_ESM * a_ESM, EC_State a_state);
};

#endif // __ethercat_fsm__
