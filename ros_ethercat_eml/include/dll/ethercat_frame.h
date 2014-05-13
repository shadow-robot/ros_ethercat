// $Id: ethercat_frame.h,v 1.21 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_frame_h__
#define __ethercat_frame_h__

#include "dll/ethercat_telegram.h"
#include "ethercat/netif.h"

/** "Dump" an EtherCAT frame into an array of bytes (for
    transmitting).
    @param frame pointer to the frame to be dumped
    @param buffer pointer to an array of unsigned chars that can be
    put into an ethernet frame
    @param bufferlength the maximum length that the frame may have
    @return the length of the buffer expressed as a number of bytes,
    if smaller than bufferlength, 0 otherwise
*/
externC int framedump(const struct EtherCAT_Frame * frame, 
		      unsigned char * buffer, 
		      size_t bufferlength);

/** Build an EtherCAT frame from an array of bytes (for receiving).
    @param frame pointer to the frame where data can be put in
    @param buffer pointer to an array of unsigned chars that contain
    the necessary info
    @return zero on succes, -1 otherwise
*/
externC int framebuild(struct EtherCAT_Frame * frame, 
		       const unsigned char * buffer);

/// EtherCAT Frame Interface (we need a C interface)
/** Due to the fact that this is a C-interface, this class does not
    herit from EC_Datastruct, although it has the same needs
*/
typedef struct EtherCAT_Frame 
{
  /// Return the length of the whole frame
  virtual size_t length(void) const = 0;
  
  /// Dump the contents of a frame into a byte array
  virtual unsigned char *   dump(unsigned char * a_buffer) const = 0;

  /// Build and check a frame from a byte array
  /** @param a_buffer the byte array
      @return 0 if succesful, -1 otherwise
  */
  virtual int build(const unsigned char * a_buffer) = 0;

  /// Get first telegram
  virtual EC_Telegram * get_telegram() const = 0;
  
} ECFrame;

/// EtherCAT Frame base class
/** @todo This interface is not complete, eg. currently there is no
    way to add/remove telegrams to a frame.  Note however that this
    EtherCAT feature is rarely used AFAIS.
*/
class EC_Frame: public ECFrame
{
 public:
  virtual ~EC_Frame();
  
  /// return the length of the whole frame
  /** @return the length of the whole frame in bytes
   */
  size_t length(void) const  { return header_length() + body_length();}
  
  /// Dump the contents of a frame into an byte array
  virtual unsigned char * dump(unsigned char * a_buffer) const;

  /// Build and check frame starting from byte array
  virtual int build(const unsigned char * a_buffer);
    
  /// Get first telegram
  EC_Telegram * get_telegram() const { return m_telegram;}
      
 protected:
  /// Constructor
  EC_Frame();
  EC_Frame(const EC_Frame & a_frame);
  EC_Frame(EC_Telegram * a_telegram);

  /// Pointer to the first EtherCAT
  EC_Telegram * m_telegram;
  
  /// Dump the header
  virtual unsigned char *    dump_header(unsigned char * a_buffer) const = 0;
  /// Check if received header in byte array complies to the frame data
  virtual bool check_header(const unsigned char * a_buffer) const = 0;

  /// Return the length of the header
  virtual size_t             header_length(void) const = 0;
  /// Return the length of the body
  virtual size_t body_length(void) const;

};

/// Ethercat Ethernet_mode frame
static const size_t ETHERCAT_ETHERNET_FRAME_HEADER_SIZE = 2;


/// Class representing EtherCAT Frames in "raw" ethernet mode
class EC_Ethernet_Frame : public EC_Frame
{
 public:
  /// Constructor
  /** @param a_telegram pointer to the first telegram of the frame */
  EC_Ethernet_Frame(EC_Telegram * a_telegram);
  virtual ~EC_Ethernet_Frame(){};

 protected:
  virtual size_t            header_length(void) const 
    { return ETHERCAT_ETHERNET_FRAME_HEADER_SIZE; }
        
  virtual unsigned char *   dump_header(unsigned char * a_buffer) const;

  virtual bool check_header(const unsigned char * a_buffer) const;
};


/// Ethercat UDP_mode Frame
// FIXME TO BE IMPLEMENTED
// class EC_UDP_Frame : public EC_Ethernet_Frame
// redefine virtual header_length() function and 

#endif // __ethercat_frame_h__
