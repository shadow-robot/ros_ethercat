// $Id: ethercat_device_addressed_telegram.h,v 1.11 2006/02/20 15:57:33 kgad Exp $
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

#ifndef __ethercat_device_addressed_telegram_h__
#define __ethercat_device_addressed_telegram_h__

#include "ros_ethercat_eml/ethercat_telegram.h"

/// EtherCAT telegram for all _device addressing_ modes

/** EtherCAT telegram for all _device addressing_ modes, including
 Position Addressing Modes (e.g.~Auto Increment Physical Read--APRD),
 Node Addressing modes (e.g.~Node addressed Physical Read--NPRD)
 and Broadcast Adressing modes (e.g.~Broadcast read)

 */
class Device_Addressing_Telegram : public EC_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp device address
   @param a_ado address offset withing device
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data
   */
  Device_Addressing_Telegram(uint8_t a_idx = 0x00,
                             uint16_t a_adp = 0x0000,
                             uint16_t a_ado = 0x0000,
                             uint16_t a_wkc = 0x0000,
                             uint16_t a_datalen = 0x0000,
                             const unsigned char * a_data = NULL);
  virtual ~Device_Addressing_Telegram()
  {
  }

  /// Set address pointer
  void set_adp(uint16_t a_adp)
  {
    m_adp = a_adp;
  }
  /// Set address offset
  void set_ado(uint16_t a_ado)
  {
    m_ado = a_ado;
  }
  /// Get address pointer
  uint16_t get_adp() const
  {
    return m_adp;
  }
  /// Get address offset
  uint16_t get_ado() const
  {
    return m_ado;
  }

protected:
  virtual unsigned char * dump_header_head(unsigned char * a_buffer) const;
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const = 0;

  virtual const unsigned char * build_header_head(const unsigned char * a_buffer);
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer) = 0;

private:
  uint16_t m_adp;
  uint16_t m_ado;
};

/// Auto Increment Physical Read Telegram (APRD)

class APRD_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp auto increment address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be read: Is actually not used, can be initialised at all zeros
   @note remove a_data parameter, since is it merely modified by the
   slave (the result of the read op)??
   */
  APRD_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

  /// Construct telegram from received bytestream...
  // APRD_Telegram(const unsigned char * a_telegram);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Auto Increment Physical Write Telegram (APWR)

class APWR_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp auto increment address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be written
   */
  APWR_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Auto Increment Physical Read and Write Telegram (APRW)

class APRW_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp auto increment address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be read and written
   */
  APRW_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Broadcast Write Telegram (BWR)

class BWR_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be written
   */
  BWR_Telegram(uint8_t a_idx, uint16_t a_ado,
               uint16_t a_wkc,
               uint16_t a_datalen,
               const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Broadcast Read Telegram (BRD)

class BRD_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be read
   */
  BRD_Telegram(uint8_t a_idx, uint16_t a_ado,
               uint16_t a_wkc,
               uint16_t a_datalen,
               const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Node Addressed Physical Write Telegram (NPWR)

/** @note older versions of the spec and the ethereal plugin use FPWR
 for this type
 */
class NPWR_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp slave address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be written
   */
  NPWR_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Node Addressed Physical Read Write Telegram (NPRW)

/** @note older versions of the spec and the ethereal plugin use FPRW
 for this type
 */
class NPRW_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp slave address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be written
   */
  NPRW_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Node Addressed Physical Read Telegram (NPRD)

/** @note older versions of the spec and the ethereal plugin use FPWR
 for this type
 */
class NPRD_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp slave address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be written
   */
  NPRD_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

/// Auto Increment Physical Read Multiple Write Telegram (ARMW)

class ARMW_Telegram : public Device_Addressing_Telegram
{
public:
  /// Constructor
  /** @param a_idx index
   @param a_adp auto increment address
   @param a_ado physical memory address
   @param a_wkc working counter
   @param a_datalen data_length
   @param a_data data to be read/written
   */
  ARMW_Telegram(uint8_t a_idx, uint16_t a_adp,
                uint16_t a_ado,
                uint16_t a_wkc,
                uint16_t a_datalen,
                const unsigned char * a_data);

protected:
  virtual unsigned char * dump_command_field(unsigned char * a_buffer) const;
  virtual const unsigned char * build_command_field(const unsigned char * a_buffer);
};

#endif // __ethercat_device_addressed_telegram_h__
