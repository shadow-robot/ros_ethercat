// $Id: ethercat_datastruct.h,v 1.5 2006/02/20 15:57:33 kgad Exp $
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

 
// fixme include copyright notice

#ifndef __ethercat_ds_h
#define __ethercat_ds_h

/// Base class for all EtherCAT register area data structs
/** The main purpose of this class is to serialize and deserialize
    data.
*/
class EC_DataStruct
{
 public:
  /// Constructor
  /** @param a_data_length data length expressed as a number of bytes
   */
  EC_DataStruct(size_t a_data_length) : m_data_length(a_data_length){};

  /// Destructor
  virtual ~EC_DataStruct(){};

  // default copy constructor will do...

  /// Return length of data array expressed as a number of bytes
  /** @return the number of bytes
   */
  size_t length() const {return m_data_length;} 

  /// Dump the data struct into an array (EtherCAT Little Endian)
  /** @param a_buffer where data should be dumped
      @return pointer just beyond the array
  */
  virtual unsigned char * dump(unsigned char * a_buffer) const = 0;

 protected:
  /// Length of data array expressed as a number of bytes
  size_t m_data_length;
};

#endif // __ethercat_ds_h
