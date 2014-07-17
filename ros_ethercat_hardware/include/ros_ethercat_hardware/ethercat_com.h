/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef ETHERCAT_COM_H
#define ETHERCAT_COM_H

#include <ros_ethercat_eml/ethercat_AL.h>
#include <ros_ethercat_eml/ethercat_master.h>
#include <ros_ethercat_eml/ethercat_slave_handler.h>
#include <ros_ethercat_eml/ethercat_dll.h>
#include <pthread.h>

class EthercatCom
{
protected:
  EthercatCom()
  {
  }

public:
  virtual bool txandrx(struct EtherCAT_Frame * frame) = 0;
  virtual bool txandrx_once(struct EtherCAT_Frame * frame) = 0;
  virtual ~EthercatCom()
  {
  }
};

class EthercatDirectCom : public EthercatCom
{
public:
  EthercatDirectCom(EtherCAT_DataLinkLayer *dll) : dll_(dll)
  {
  }

  bool txandrx(struct EtherCAT_Frame * frame);
  bool txandrx_once(struct EtherCAT_Frame * frame);

protected:
  EtherCAT_DataLinkLayer *dll_;
};

class EthercatOobCom : public EthercatCom
{
public:
  EthercatOobCom(struct netif *ni);

  bool txandrx(struct EtherCAT_Frame * frame);
  bool txandrx_once(struct EtherCAT_Frame * frame);

  void tx();
protected:
  bool lock(unsigned line);
  bool trylock(unsigned line);
  bool unlock(unsigned line);

  struct netif *ni_;
  pthread_mutex_t mutex_;
  pthread_cond_t share_cond_;
  pthread_cond_t busy_cond_;

  enum
  {
    IDLE = 0, READY_TO_SEND = 1, WAITING_TO_RECV = 2
  } state_;
  EtherCAT_Frame *frame_;
  int handle_;
  unsigned line_;
};

#endif /* ETHERCAT_COM_H */
