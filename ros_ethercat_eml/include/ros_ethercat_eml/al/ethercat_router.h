// $Id: ethercat_router.h,v 1.14 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_router__
#define __ethercat_router__

// forward declaration
class EtherCAT_AL;
class EC_Logic;
class EtherCAT_DataLinkLayer;
class EtherCAT_SlaveHandler;
class EtherCAT_MbxMsg;

/// EtherCAT Router component
class EtherCAT_Router
{
 public:
  /// Singleton
  static EtherCAT_Router * instance();
  virtual ~EtherCAT_Router();

  /// Start routing
  void start();
  /// Stop routing
  void stop();
  /// Router running?
  bool is_running() const { return (EtherCAT_Router::m_is_running != 0); }

  /// Actual routing code
  /** @todo  This method should not be public, but since it is used in
      a C-function and I don't know how I could declare a C-function
      to be a friend of a class...
      @todo  The current implementation simply polls the sync manager
      data area of each slave and checks if wkc is incremented.
      Another solution is to configure FMMU's of each slave and query
      the slaves with one logical EtherCAT msg.  This has the
      advantage of only using one message per slave, but it
      "sacrifices" an extra FMMU per slave, which might not always be
      possible.  In that case, the master would---during the
      start_mbx_communication() call of slaves---configure an FMMU of
      the slave which listens to the written bit of sync manager 0.
   */
  void route(void) const;  
 protected:
  EtherCAT_Router();
  
 private:
  /// Pointer to AL instance
  EtherCAT_AL * m_al_instance;
  /// Pointer to EC_Logic
  EC_Logic * m_logic_instance;
  /// Pointer to DLL instance
  EtherCAT_DataLinkLayer * m_dll_instance;
  
  static EtherCAT_Router * m_instance;
  /// Usage counter
  unsigned int m_is_running;

  /// Check and deliver post from slave
  /** Check if slave posted something in its mailbox and if positive,
      deliver msgs to destination.
      @param sh Slave Handler to query
      @return true if nothing went wrong (i.e. nothing was posted, or
      no error occurred during routing
  */
  bool check_mbx(const EtherCAT_SlaveHandler * sh) const;
  
  /// Post a msg from a certain slave
  /** @param msg mbx_msg to be posted
      @param from_sh "poster" of the msg (The EtherCAT protocol does
      not provide a From: header in its messages)
      @return true if everything went fine
   */
  bool post_mbxmsg(EtherCAT_MbxMsg * msg, 
		   const EtherCAT_SlaveHandler * from_sh) const;

};

#endif
