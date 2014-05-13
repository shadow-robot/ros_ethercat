// $Id: ethercat_slave_memory.h,v 1.25 2006/02/20 15:57:33 kgad Exp $
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

 
#ifndef __ethercat_slave_memory_h__
#define __ethercat_slave_memory_h__

#include "ethercat/ethercat_defs.h"
#include "ethercat/ethercat_log.h"
#include "ethercat/ethercat_datastruct.h"
#include <stdio.h>
#include <cassert>

// Describes registers of EtherCAT slave controller
static const size_t EC_DLInformationSize = 0xa; // 10 bytes

// Product code and revision offset in SII Eeprom data 
/* BIG FAT WARNING:  THIS VARIES FROM WHAT IS WRITTEN IN THE ETHERCAT
   SPEC VERSION 1.0!!!
*/
static const EC_UDINT EC_ProductCodeAddressInSII = 0x0000000a;
static const EC_UDINT EC_RevisionAddressInSII = 0x0000000c;
static const EC_UDINT EC_SerialAddressInSII = 0x0000000e;

typedef enum {
  Type = 0,
  Revision,
  Build,
  Num_FMMUs,
  Num_Sync_Managers,
  RAM_Size,
  ECAT_Station_Address,
  DLL_Control,
  DLL_Status,
  AL_Control,
  AL_Status,
  PDI_Control,
  PDI_Conf_reg,
  AL_Event,
  RX_Error_Counter_Channel_A,
  RX_Error_Counter_Channel_B,
  Watchdog_Divider,
  Watchdog_Time_PDI,
  Watchdog_Time_Channel_0,
  Watchdog_Time_Channel_1,
  Watchdog_Time_Channel_2,
  Watchdog_Time_Channel_3,
  Watchdog_Time_Channel_4,
  Watchdog_Time_Channel_5,
  Watchdog_Time_Channel_6,
  Watchdog_Time_Channel_7,
  Watchdog_Time_Channel_8,
  Watchdog_Time_Channel_9,
  Watchdog_Time_Channel_10,
  Watchdog_Time_Channel_11,
  Watchdog_Time_Channel_12,
  Watchdog_Time_Channel_13,
  Watchdog_Time_Channel_14,
  Watchdog_Time_Channel_15,
  Watchdog_Channel_status,
  SII_Size,
  SII_ControlStatus,
  SII_Address,
  SII_Data,
  FMMU_0,
  FMMU_1,
  FMMU_2,
  FMMU_3,
  FMMU_4,
  FMMU_5,
  FMMU_6,
  FMMU_7,
  FMMU_8,
  FMMU_9,
  FMMU_10,
  FMMU_11,
  FMMU_12,
  FMMU_13,
  FMMU_14,
  FMMU_15,
  Sync_Manager_0,
  Sync_Manager_1,
  Sync_Manager_2,
  Sync_Manager_3,
  Sync_Manager_4,
  Sync_Manager_5,
  Sync_Manager_6,
  Sync_Manager_7,
  Sync_Manager_8,
  Sync_Manager_9,
  Sync_Manager_10,
  Sync_Manager_11,
  Sync_Manager_12,
  Sync_Manager_13,
  Sync_Manager_14,
  Sync_Manager_15,
  NumRegisters,
} ECAT_Slave_Registers;

typedef enum {
  EC_R = 0, // Read access only
  EC_W = 1, // Write access only
  EC_RW = 2 // RW accesc
} ECAT_Register_Access;



/// EtherCAT Slave Register data
typedef struct
{
  /// Register name
  const char * name; 
  /// Register offset addres
  const EC_UINT ado; 
  /// Access of the master to the register (r, w, or r/w)
  const ECAT_Register_Access ECAT_access;
  /// Access of the slave to the register (r, w, or r/w)
  const ECAT_Register_Access uC_access;
  /// Size of the register area (expressed as number of bytes)
  const EC_USINT size; 
} ECAT_Slave_Register_Data;



static const ECAT_Slave_Register_Data EC_Slave_RD[NumRegisters] = {
  {"Type", 0x0000, EC_R, EC_R, 0x01},
  {"Revision", 0x0001, EC_R, EC_R, 0x01},
  {"Build",0x0002, EC_R, EC_R, 0x02},
  {"Number of FMMUs",0x0004,EC_R, EC_R, 0x01},
  {"Number of Sync Managers",0x0005,EC_R, EC_R, 0x01},
  {"RAM Size",0x0006,EC_R, EC_R, 0x01},
  {"EtherCAT Station Address",0x0010,EC_RW,EC_R,0x02},
  {"DLL Control Register",0x0100,EC_RW,EC_R,0x02},
  {"DLL Status Register",0x0110,EC_R,EC_R,0x02},
  {"AL Control Register",0x0120,EC_RW,EC_R,0x02},
  {"AL Status Register",0x0130,EC_R,EC_RW,0x02},
  {"PDI Control Register",0x0140,EC_R,EC_R,0x02},
  {"PDI Config Register",0x0150,EC_R,EC_R,0x05},
  {"AL Event Register",0x0220,EC_R,EC_R,0x04},
  {"RX Error Counter Channel A",0x0300,EC_RW,EC_R,0x02},
  {"RX Error Counter Channel B",0x0302,EC_RW,EC_R,0x02},
  {"Watchdog divider",0x0400,EC_RW,EC_R,0x02},
  {"Watchdog Time PDI",0x0410,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 0",0x0420,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 1",0x0422,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 2",0x0424,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 3",0x0426,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 4",0x0428,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 5",0x042a,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 6",0x042c,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 7",0x042e,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 8",0x0430,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 9",0x0432,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 10",0x0434,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 11",0x0436,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 12",0x0438,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 13",0x043a,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 14",0x043c,EC_RW,EC_R,0x02},
  {"Watchdog Time Channel 15",0x043e,EC_RW,EC_R,0x02},
  {"Watchdog Channel status",0x0450,EC_R,EC_R,0x02},
  {"SII Eeprom Size",0x0500,EC_R,EC_R,0x02},
  {"SII Control/Status",0x0502,EC_RW,EC_R,0x02},
  {"SII Address",0x0504,EC_RW,EC_R,0x04},
  {"SII Data",0x0508,EC_RW,EC_R,0x04},
  {"FMMU0",0x0600,EC_RW,EC_R,0x10}, 
  {"FMMU1",0x0610,EC_RW,EC_R,0x10}, 
  {"FMMU2",0x0620,EC_RW,EC_R,0x10}, 
  {"FMMU3",0x0630,EC_RW,EC_R,0x10}, 
  {"FMMU4",0x0640,EC_RW,EC_R,0x10}, 
  {"FMMU5",0x0650,EC_RW,EC_R,0x10}, 
  {"FMMU6",0x0660,EC_RW,EC_R,0x10}, 
  {"FMMU7",0x0670,EC_RW,EC_R,0x10}, 
  {"FMMU8",0x0680,EC_RW,EC_R,0x10}, 
  {"FMMU9",0x0690,EC_RW,EC_R,0x10}, 
  {"FMMU10",0x06A0,EC_RW,EC_R,0x10},
  {"FMMU11",0x06B0,EC_RW,EC_R,0x10},
  {"FMMU12",0x06C0,EC_RW,EC_R,0x10},
  {"FMMU13",0x06D0,EC_RW,EC_R,0x10},
  {"FMMU14",0x06E0,EC_RW,EC_R,0x10},
  {"FMMU15",0x06F0,EC_RW,EC_R,0x10},
  {"Sync Manager 0",0x0800,EC_RW,EC_R,0x08},
  {"Sync Manager 1",0x0808,EC_RW,EC_R,0x08},
  {"Sync Manager 2",0x0810,EC_RW,EC_R,0x08},
  {"Sync Manager 3",0x0818,EC_RW,EC_R,0x08},
  {"Sync Manager 4",0x0820,EC_RW,EC_R,0x08},
  {"Sync Manager 5",0x0828,EC_RW,EC_R,0x08},
  {"Sync Manager 6",0x0830,EC_RW,EC_R,0x08},
  {"Sync Manager 7",0x0838,EC_RW,EC_R,0x08},
  {"Sync Manager 8",0x0840,EC_RW,EC_R,0x08},
  {"Sync Manager 9",0x0848,EC_RW,EC_R,0x08},
  {"Sync Manager 10",0x0850,EC_RW,EC_R,0x08},
  {"Sync Manager 11",0x0858,EC_RW,EC_R,0x08},
  {"Sync Manager 12",0x0860,EC_RW,EC_R,0x08},
  {"Sync Manager 13",0x0868,EC_RW,EC_R,0x08},
  {"Sync Manager 14",0x0870,EC_RW,EC_R,0x08},
  {"Sync Manager 15",0x0878,EC_RW,EC_R,0x08}
};


static inline int FMMUx(int channel)
{
  switch(channel) {
  case 0 : return FMMU_0; break;
  case 1 : return FMMU_1; break;
  case 2 : return FMMU_2; break;
  case 3 : return FMMU_3; break;
  case 4 : return FMMU_4; break;
  case 5 : return FMMU_5; break;
  case 6 : return FMMU_6; break;
  case 7 : return FMMU_7; break;
  case 8 : return FMMU_8; break;
  case 9 : return FMMU_9; break;
  case 10 : return FMMU_10; break;
  case 11 : return FMMU_11; break;
  case 12 : return FMMU_12; break;
  case 13 : return FMMU_13; break;
  case 14 : return FMMU_14; break;
  case 15 : return FMMU_15; break;
  default:
    ec_log(EC_LOG_ERROR, "FMMUx: No such channel %d\n",channel);
    return -1;
  }
}

static inline int Sync_Managerx(int channel)
{
  switch(channel) {
  case 0 : return Sync_Manager_0; break;
  case 1 : return Sync_Manager_1; break;
  case 2 : return Sync_Manager_2; break;
  case 3 : return Sync_Manager_3; break;
  case 4 : return Sync_Manager_4; break;
  case 5 : return Sync_Manager_5; break;
  case 6 : return Sync_Manager_6; break;
  case 7 : return Sync_Manager_7; break;
  case 8 : return Sync_Manager_8; break;
  case 9 : return Sync_Manager_9; break;
  case 10 : return Sync_Manager_10; break;
  case 11 : return Sync_Manager_11; break;
  case 12 : return Sync_Manager_12; break;
  case 13 : return Sync_Manager_13; break;
  case 14 : return Sync_Manager_14; break;
  case 15 : return Sync_Manager_15; break;
  default:
    ec_log(EC_LOG_ERROR, "Sync_Managerx: No such channel %d\n",channel);
    return -1;
  }
}

static inline int Watchdog_Time_Channelx(int channel)
{
  switch(channel) {
  case 0 : return Watchdog_Time_Channel_0; break;
  case 1 : return Watchdog_Time_Channel_1; break;
  case 2 : return Watchdog_Time_Channel_2; break;
  case 3 : return Watchdog_Time_Channel_3; break;
  case 4 : return Watchdog_Time_Channel_4; break;
  case 5 : return Watchdog_Time_Channel_5; break;
  case 6 : return Watchdog_Time_Channel_6; break;
  case 7 : return Watchdog_Time_Channel_7; break;
  case 8 : return Watchdog_Time_Channel_8; break;
  case 9 : return Watchdog_Time_Channel_9; break;
  case 10 : return Watchdog_Time_Channel_10; break;
  case 11 : return Watchdog_Time_Channel_11; break;
  case 12 : return Watchdog_Time_Channel_12; break;
  case 13 : return Watchdog_Time_Channel_13; break;
  case 14 : return Watchdog_Time_Channel_14; break;
  case 15 : return Watchdog_Time_Channel_15; break;
  default:
    ec_log(EC_LOG_ERROR, "Watchdog_Time_Channelx: No such channel %d\n",channel);
    return -1;
  }
}

/// EtherCAT Data Layer Information 
class EC_DLInformation : public EC_DataStruct
{
 public:
  EC_DLInformation(EC_USINT type,
		   EC_USINT revision,
		   EC_UINT build,
		   EC_USINT no_of_supp_fmmu_channels,
		   EC_USINT no_of_supp_syncman_channels,
		   EC_USINT ram_size,
		   bool fmmu_bit_operation_not_supp);
  EC_DLInformation(const unsigned char * a_buffer);
  virtual ~EC_DLInformation(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;

  EC_USINT Type;
  EC_USINT Revision;
  EC_UINT Build;
  EC_USINT NoOfSuppFmmuChannels;
  EC_USINT NoOfSuppSyncManChannels;
  EC_USINT RamSize;
  // EC_USINT Reserved1;
  bool FmmuBitOperationNotSupp;
  // EC_UINT Reserved2;
  // EC_USINT Reserved3;
};

/// EtherCAT Fixed Station Address
class EC_FixedStationAddress : public EC_DataStruct
{
 public:
  EC_FixedStationAddress(EC_UINT fixed_station_address = 0x0000) :
    EC_DataStruct(EC_Slave_RD[ECAT_Station_Address].size), FixedStationAddress(fixed_station_address){};
  virtual ~EC_FixedStationAddress(){};
  EC_FixedStationAddress(const unsigned char * data) :
    EC_DataStruct(EC_Slave_RD[ECAT_Station_Address].size) { nw2host(data,FixedStationAddress); }

  virtual unsigned char * dump(unsigned char * a_buffer) const { return host2nw(a_buffer, FixedStationAddress); }

  void operator = (const EC_FixedStationAddress & ad){
    this->FixedStationAddress = ad.FixedStationAddress;
  }
  bool operator ==(const EC_FixedStationAddress & ad) const{
    return (this->FixedStationAddress == ad.FixedStationAddress);
  }
  operator EC_UINT() const { return FixedStationAddress; }
 private:
  /// Fixed Station Address
  EC_UINT FixedStationAddress;
};
 
/* class EC_DLControl : public EC_DataStruct */
/* { */
/*   // fixme todo   */
/* }; */


/* class EC_DLStatus : public EC_DataStruct */
/* { */
/* }; */

// EtherCAT states
typedef enum 
{
  EC_INIT_STATE = 0x01,
  EC_PREOP_STATE = 0x02,
  EC_BOOTSTRAP_STATE = 0x03,
  EC_SAFEOP_STATE = 0x04,
  EC_OP_STATE = 0x08
} EC_State;

/// AL Control register
class EC_ALControl : public EC_DataStruct
{
 public:
  /// Constructor
  /** @param state EtherCAT state:  Can be one of the following:
      EC_INIT_STATE,  EC_PREOP_STATE, EC_BOOTSTRAP_STATE,
      EC_SAFEOP_STATE, EC_OP_STATE 
      @param ack Acknowledge
      @note The bootstrap state is currently unimplemented
   */
  EC_ALControl(EC_State state = EC_INIT_STATE, 
	       bool ack       =false);
  /// Constructor: build from data array
  EC_ALControl(const unsigned char * a_buffer);
  virtual ~EC_ALControl(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;

  EC_State State; // 4 bits
  bool Acknowledge; // 1 bit
  // EC_USINT ApplSpecific; // 8 bits
};

/// AL Status register
class EC_ALStatus : public EC_DataStruct
{
 public:
  /// Constructor
  /** @param state EtherCAT state:  Can be one of the following:
      EC_INIT_STATE,  EC_PREOP_STATE, EC_BOOTSTRAP_STATE,
      EC_SAFEOP_STATE, EC_OP_STATE 
      @param change Change
   */
  EC_ALStatus(EC_State state = EC_INIT_STATE, 
	      bool change    = false);
  /// Constructor: build from data array
  EC_ALStatus(const unsigned char * a_buffer);
  virtual ~EC_ALStatus(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;

  EC_State State; // 4 bits
  bool Change; // 1 bit
  // EC_USINT ApplSpecific; // 8 bits
};

/* class EC_PDIControl : public EC_DataStruct */
/* { */
/* }; */

// fixme EC_PDIConfigurationMCI16
// fixme EC_PDIConfigurationMCI16
// fixme EC_ALEvent
// fixme statistics and watchdogs

/// Slave Information Interface Control/Status
class EC_SIIControlStatus : public EC_DataStruct
{
 public:
  /// Constructor (see spec for args)
  EC_SIIControlStatus(bool eeprom_write_access = false,
		      bool eeprom_address_algorithm = false,
		      bool read_op = false,
		      bool write_op = false,
		      bool reload_op = false,
		      bool write_error = false,
		      bool busy = false);
  /// Constructor: build from data array
  EC_SIIControlStatus(const unsigned char * a_buffer);
  virtual ~EC_SIIControlStatus(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;
  
  /// Write access?
  bool EepromWriteAccess;
  /// ?? FIXME meaning?
  bool EepromAddressAlgorithm; 
  /// Read Operation?
  bool ReadOp;
  /// Write Operation?
  bool WriteOp;
  /// Reload Operation?
  bool ReloadOp; 
  /// Write Error?
  bool WriteError;
  /// Eeprom busy?
  bool Busy;
  // Acknowledge Error
  bool AcknowledgeError;
};

/// Class representing a bit position within a byte
class EC_BitPos
{
public:
  /// Constructor
  /** @param a_int Bit position in the byte (an int from 0 to 7)
   */
  EC_BitPos(EC_USINT a_int = 0){
    // There are only 8 bits in a byte...
    assert(a_int < 8);
    m_bitpos = a_int;
  }
  virtual ~EC_BitPos(){};
  /// Cast operator
  operator EC_USINT() const {return m_bitpos;}
private:
  EC_USINT m_bitpos;
};

/// EtherCAT FMMU
class EC_FMMU : public EC_DataStruct
{
 public:
  /// Constructor (see spec for params)
  EC_FMMU(EC_UDINT  logical_start_address = 0x00000000,
	  EC_UINT   length                = 0x0000,
	  EC_BitPos logical_start_bit     = 0x00,
	  EC_BitPos logical_end_bit       = 0x00,
	  EC_UINT   physical_start_address= 0x0000,
	  EC_BitPos physical_start_bit    = 0x00,
	  bool      read_enable           = false,
	  bool      write_enable          = false,
	  bool      channel_enable        = false);
  /// Constructor: build from data array
  EC_FMMU(const unsigned char * a_buffer);
  virtual ~EC_FMMU(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;
  
  EC_UDINT  LogicalStartAddress;
  EC_UINT   Length;
  EC_BitPos LogicalStartBit; 
  // EC_USINT  Reserved1; // 5 bits
  EC_BitPos LogicalEndBit;
  // EC_UINT   Reserved2; // 5 bits;
  EC_UINT   PhysicalStartAddress;
  EC_BitPos PhysicalStartBit;
  // EC_USINT  Reserved3; // 5 bits
  bool ReadEnable;
  bool WriteEnable;
  // EC_USINT  Reserved4; // 6 bits
  bool ChannelEnable;
  // EC_USINT  Reserved5; // 7 bits
  // EC_USINT Reserved6; // 1 byte
  // EC_UINT  Reserved7; // 2 bytes
};

// ==================================================

typedef enum
{
  EC_BUFFERED = 0x00,
  EC_QUEUED = 0x02,
} ECBufferType;

/// Class representing queued or buffered Syncman buffertypes
/** @see EC_SyncMan
 */
class EC_BufferType
{
 public:
  /// Constructor
  /** @param bt Buffertype (buffered (EC_BUFFERED) or queued (EC_QUEUED)
   */
  EC_BufferType(ECBufferType bt = EC_BUFFERED){
    m_buffertype = (EC_USINT) bt;
  }
  virtual ~EC_BufferType(){};
  /// Cast operator
  operator EC_USINT() const {return m_buffertype;}
private:
  EC_USINT m_buffertype;
};

typedef enum
{
  EC_READ_FROM_MASTER = 0x00,
  EC_WRITTEN_FROM_MASTER = 0x01,
} ECDirection;

/// Class representing read/write direction of Syncman buffer
/** @see EC_SyncMan
 */
class EC_Direction
{
 public:
  /// Constructor
  /** @param dir R/W direction of Sync Manager Channel.  Possible
      values are
      - "Memory is read from Master" (EC_READ_FROM_MASTER) and 
      - "Memory is written from Master" (EC_WRITTEN_FROM_MASTER) 
  */
  EC_Direction(ECDirection dir = EC_READ_FROM_MASTER){
    m_direction = (EC_USINT) dir;
  }
  virtual ~EC_Direction(){};
  /// Cast operator
  operator EC_USINT() const {return m_direction;}
private:
  EC_USINT m_direction;
};

typedef enum {
  EC_FIRST_BUFFER = 0x00,
  EC_SECOND_BUFFER = 0x01,
  EC_THIRD_BUFFER = 0x02,
  EC_LOCKED_BUFFER = 0x03,
} ECBufferedState;

/// Class representing buffered state of a SyncMan
/** @see EC_SyncMan
 */
class EC_BufferedState
{
 public:
  /// Constructor
  /** @param state Buffered state of the Sync Manager Channel.
      Possible values are
      - EC_FIRST_BUFFER, 
      - EC_SECOND_BUFFER,
      - EC_THIRD_BUFFER, 
      - EC_LOCKED_BUFFER
  */
  EC_BufferedState(ECBufferedState state = EC_FIRST_BUFFER){
    m_buffered_state = (EC_USINT) state;
  }
  virtual ~EC_BufferedState(){};
  /// Cast operator
  operator EC_USINT() const {return m_buffered_state;}
private:
  EC_USINT m_buffered_state;
};

static const bool EC_QUEUED_STATE_READ = false;
static const bool EC_QUEUED_STATE_WRITTEN = true;

/// EtherCAT Sync Manager
class EC_SyncMan : public EC_DataStruct
{
 public:
  /// Constructor (see spec for arguments)
  EC_SyncMan(EC_UINT physical_start_address = 0x0000,
	     EC_UINT length                 = 0x0000,
	     EC_BufferType buffer_type      = EC_BUFFERED,
	     EC_Direction direction         = EC_READ_FROM_MASTER,
	     bool AL_event_enable           = false,
	     bool watchdog_enable           = false,
	     bool write_event               = false,
	     bool read_event                = false,
             bool watchdog_trigger          = false,
	     bool queued_state              = EC_QUEUED_STATE_READ,
	     EC_BufferedState buffered_state= EC_FIRST_BUFFER,
	     bool ChannelEnable             = false);
  /// Constructor: build from data array
  EC_SyncMan(const unsigned char * a_buffer);
  virtual ~EC_SyncMan(){};
  virtual unsigned char * dump(unsigned char * a_buffer) const;
  
  EC_UINT PhysicalStartAddress;
  EC_UINT Length;
  EC_BufferType BufferType;
  EC_Direction Direction;
  bool ALEventEnable;
  bool ECATEventEnable;
  bool WatchdogEnable;
  bool WriteEvent;
  bool ReadEvent;
  bool WatchdogTrigger;
  bool QueuedState;
  EC_BufferedState BufferedState;
  bool ChannelEnable;
};

// fixme distributed clock stuff

#endif // __ethercat_slave_memory_h__
