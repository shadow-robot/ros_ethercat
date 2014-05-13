// $Id: ethercat_mbx.h,v 1.14 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_mbx__
#define __ethercat_mbx__

#include "ethercat/ethercat_defs.h"
#include "dll/ethercat_slave_memory.h"

typedef enum {
  EC_AoE = 0x01, // ADS over EtherCAT
  EC_EoE = 0x02, // Ethernet over EtherCAT
  EC_CoE = 0x03, // CANOpen over EtherCAT
  EC_FoE = 0x04, // File Access over EtherCAT
} ECMbxMsgType;

/// EtherCAT Mbx Message type
class EC_MbxMsgType
{
 public:
  /// Constructor
  /** @param type Message type:  Possibilities include
      - EC_AoE (ADS over EtherCAT),
      - EC_EoE (Ethernet over EtherCAT)
      - EC_CoE (CANOpen over EtherCAT)
      - EC_FoE (File Access over EtherCAT)
  */
  EC_MbxMsgType(ECMbxMsgType type = EC_CoE) : msg_type(type){};
  virtual ~EC_MbxMsgType(){};
  /// Cast operator
  operator EC_USINT() const {return msg_type;}
 private:
  EC_USINT msg_type;
};

/// EtherCAT Mbx Message Priority
class EC_MbxMsgPriority
{
 public:
  /// Constructor
  /** @param priority priority of the msg (from 0 to 3 is allowed,
      with 3 the max Priority)
  */
  EC_MbxMsgPriority(EC_USINT priority = 0x00)
    {
      if(priority < 4)
	msg_priority = priority;
      else {
	ec_log(EC_LOG_WARNING, "EC_MbxMsgPriority: Max Priority is 0x03, using 0x03\n");
	msg_priority = 0x03;
      }
    }
  virtual ~EC_MbxMsgPriority(){};
  /// Cast operator
  operator EC_USINT() const {return msg_priority;}
 private:
  EC_USINT msg_priority;
};

// Header size is 6 bytes
static const size_t EC_MBXMSG_HDR_SIZE = 6;

/// EtherCAT MbxMsg header
class EC_MbxMsgHdr : public EC_DataStruct
{
  friend class EtherCAT_Router;
  
 public:
  /// Constructor
  /** @param a_length Length of the MbxMsg
      @param a_address 
      - For Master/Slave and Slave/Master communication: Address of
      destination.
      - For slave to slave communication (routing by Master).  Slave
        puts station address of destination (it must therefore now this
	address!) so master knows where to send it.  Master (in the
	router) alters address field so destination slave knows where
	the message came from.
      @param a_priority Priority of this message
      @param a_type Type of this Message (higher level protocol)
  */
  EC_MbxMsgHdr(EC_UINT a_length, 
		EC_FixedStationAddress a_address,
		EC_MbxMsgPriority a_priority,
		EC_MbxMsgType a_type)
    : EC_DataStruct(EC_MBXMSG_HDR_SIZE),
    m_length(a_length), m_address(a_address), 
    m_priority(a_priority), m_type(a_type){};
  
  /// Constructor from data array
  EC_MbxMsgHdr(const unsigned char * a_buffer);
  virtual ~EC_MbxMsgHdr(){};

  virtual unsigned char * dump(unsigned char * a_buffer) const;

 public:
  EC_UINT  m_length;
  EC_FixedStationAddress m_address;
  // EC_USINT  m_channel; 6 bits unused for now...;
  EC_MbxMsgPriority  m_priority;
  EC_MbxMsgType      m_type;
  // EC_UINT reserved : 4;
};

/// EtherCAT Mailbox Message
class EtherCAT_MbxMsg
{
  friend class EtherCAT_Router;
  
 public:
  /// Constructor
  EtherCAT_MbxMsg(EC_MbxMsgHdr a_hdr, unsigned char * a_MbxMsgdata)
    : m_hdr(a_hdr), m_MbxMsgdata(a_MbxMsgdata){}
  EtherCAT_MbxMsg(EC_UINT a_length, EC_UINT a_address,
		  EC_MbxMsgPriority a_priority,
		  EC_MbxMsgType a_type,
		  unsigned char * a_MbxMsgdata)
    : m_hdr(a_length,a_address,a_priority,a_type), m_MbxMsgdata(a_MbxMsgdata){}
  EtherCAT_MbxMsg(const unsigned char * a_buffer);

  virtual ~EtherCAT_MbxMsg(){};

  /// Dump msg to buffer for sending...
  virtual unsigned char * dump(unsigned char * a_buffer) const;

 protected:
  /// MbxMsg header
  EC_MbxMsgHdr m_hdr;
  /// MbxMsg data
  const unsigned char * m_MbxMsgdata;

  virtual unsigned char * dump_data(unsigned char * a_buffer) const;
};

/// EtherCAT Mailbox
class EtherCAT_Mbx
{
 public:
  /// Write a message to the mailbox
  bool write(EtherCAT_MbxMsg * a_msg);
  /// Read a message from the mailbox
  bool read(EtherCAT_MbxMsg * a_msg);  

  EtherCAT_Mbx(){};
  virtual ~EtherCAT_Mbx(){};
 protected:
};

// ==================================================
// CANopen over EtherCAT
// ==================================================

typedef enum {
  CANopen_Emergency = 0x01,
  CANopen_SDORequest = 0x02,
  CANopen_SDOResponse = 0x03,
  CANopen_txPDO = 0x04,
  CANopen_rxPDO = 0x05,
  CANopen_txPDORemoteReq = 0x06,
  CANopen_rxPDORemoteReq = 0x07,
  CANopen_SDOInformation = 0x08,
} CANopenService;

/// CANOpen Service
class CANopen_Service
{
 public:
  /// Constructor
  /** @param a_service CanOpen Service.  Possibilities are
      - CANopen_Emergency, 
      - CANopen_SDORequest, 
      - CANopen_SDOResponse,
      - CANopen_txPDO, 
      - CANopen_rxPDO, 
      - CANopen_txPDORemoteReq,
      - CANopen_rxPDORemoteReq, 
      - CANopen_SDOInformation
   */
  CANopen_Service(CANopenService a_service = CANopen_Emergency)
    : m_service(a_service){}

  virtual ~CANopen_Service(){};
  /// Cast operator
  operator EC_USINT() const { return m_service; }

 private:
  CANopenService m_service;
};

static const size_t EC_MBXMSG_COE_HDR_SIZE = 2;

/// CANopen over EtherCAT Mailbox Message header
class EC_CoE_Hdr : public EC_DataStruct
{
 public:
  /// Constructor
  /** @param a_service CanOpen Service
  */
  EC_CoE_Hdr(CANopen_Service a_service = CANopen_Emergency) :
    EC_DataStruct(EC_MBXMSG_COE_HDR_SIZE),
    m_service(a_service){};
  EC_CoE_Hdr(const unsigned char * a_buffer);
  virtual ~EC_CoE_Hdr(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;

 private:
  // FIXME meaning of these is nog clear from spec
  // Number Low: 8 bits depending on CANopen service
  // Number High: 1 bit depending on CANopen service
  CANopen_Service m_service;
};

/// CANopen over EtherCAT Mailbox Message
class EtherCAT_CoE_MbxMsg : public EtherCAT_MbxMsg
{
 public:
  EtherCAT_CoE_MbxMsg(EC_MbxMsgHdr a_hdr, 
		      EC_CoE_Hdr a_CoE_hdr,
		      unsigned char * a_MbxMsgdata)
    : EtherCAT_MbxMsg(a_hdr,a_MbxMsgdata), m_CoE_Hdr(a_CoE_hdr){}
  EtherCAT_CoE_MbxMsg(EC_UINT a_length, EC_UINT a_address,
		      EC_MbxMsgPriority a_priority,
		      EC_MbxMsgType a_type,
		      CANopen_Service a_service,
		      unsigned char * a_MbxMsgdata)
    : EtherCAT_MbxMsg(a_length,a_address,a_priority,a_type,a_MbxMsgdata), m_CoE_Hdr(a_service){}
  EtherCAT_CoE_MbxMsg(unsigned char * a_buffer);

  virtual unsigned char * dump(unsigned char * a_buffer) const;
 protected:
  EC_CoE_Hdr m_CoE_Hdr;
};

#endif // __ethercat_mbx__
