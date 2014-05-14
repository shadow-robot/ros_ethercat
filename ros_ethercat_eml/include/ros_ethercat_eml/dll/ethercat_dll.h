// $Id: ethercat_dll.h,v 1.22 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_dll__
#define __ethercat_dll__

// Forward declarations
struct netif;
class EtherCAT_Frame;

/// Abstract representation of the EtherCAT DLL
class EtherCAT_DataLinkLayer
{
  public:
  /// This class is a singleton
  /** @todo Pass destination MAC address instead of defaulting to
      broadcast?
  */
  static EtherCAT_DataLinkLayer * instance(void);

  /// Attach the DLL to a networking interface
  /** @param netif pointer to the NIC
   */
  void                attach(struct netif * netif);

  /// transmit and receive EtherCAT frame (blocking call!)  
  /** @param a_frame ethercat frame to be sent
   *  NOTE that txandrx will retry sending lost frames.
  */
  bool                txandrx(EtherCAT_Frame * a_frame);

  /// transmit an EtherCAT frame (non-blocking call)
  /** @param a_frame ethercat frame to be sent
   *  @return positive or zero handle on success, negative value for error 
   *  Successfull tx MUST be followed by rx call.
   */
  int tx(EtherCAT_Frame * a_frame);

  /// receive an EtherCAT frame previously sent by tx() (blocking call!)
  /** @param a_frame ethercat frame to be sent
   *  @param a_handle handle previously returned by tx()
   *  @return false for error/dropped packet
   */
  bool rx(EtherCAT_Frame * a_frame, int a_handle);

  /// Destructor
  /** @todo some kind of smart pointer concept necessary for all these
      singletons.  For EtherCAT master on ecos not really an issue...
  */
  virtual            ~EtherCAT_DataLinkLayer();

  protected:
                        EtherCAT_DataLinkLayer();

  private:
    static EtherCAT_DataLinkLayer * m_instance;
    struct netif *                  m_if;
};

#endif // __ethercat_dll__
