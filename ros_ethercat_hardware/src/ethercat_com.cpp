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

#include "ros_ethercat_hardware/ethercat_com.h"
#include <stdio.h>
#include <errno.h>
#include <assert.h>

bool EthercatDirectCom::txandrx_once(struct EtherCAT_Frame * frame)
{
  assert(frame != NULL);
  int handle = dll_->tx(frame);
  if (handle < 0)
    return false;
  return dll_->rx(frame, handle);
}

bool EthercatDirectCom::txandrx(struct EtherCAT_Frame * frame)
{
  return dll_->txandrx(frame);
}

EthercatOobCom::EthercatOobCom(struct netif *ni) :
  ni_(ni),
  state_(IDLE),
  frame_(NULL),
  handle_(-1),
  line_(0)
{
  assert(ni_ != NULL);

  pthread_mutexattr_t mutex_attr;
  int error = pthread_mutexattr_init(&mutex_attr);
  if (error != 0)
  {
    fprintf(stderr, "%s : Initializing mutex attr failed : %d\n", __func__, error);
    return;
  }
  error = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK_NP);
  if (error != 0)
  {
    fprintf(stderr, "%s : Setting type of mutex attr failed : %d\n", __func__, error);
    return;
  }
  error = pthread_mutex_init(&mutex_, &mutex_attr);
  if (error != 0)
  {
    fprintf(stderr, "%s : Initializing mutex failed : %d\n", __func__, error);
    return;
  }
  error = pthread_cond_init(&share_cond_, NULL);
  if (error != 0)
  {
    fprintf(stderr, "%s : Initializing share condition failed : %d\n", __func__, error);
    return;
  }
  error = pthread_cond_init(&busy_cond_, NULL);
  if (error != 0)
    fprintf(stderr, "%s : Initializing busy condition failed : %d\n", __func__, error);
  return;
}

bool EthercatOobCom::lock(unsigned line)
{
  int error;
  if (0 != (error = pthread_mutex_lock(&mutex_)))
  {
    fprintf(stderr, "%s : lock %d at %d\n", __func__, error, line);
    return false;
  }
  line_ = line;
  return true;
}

bool EthercatOobCom::trylock(unsigned line)
{
  int error;
  if (0 != (error = pthread_mutex_trylock(&mutex_)))
  {
    if (error != EBUSY)
      fprintf(stderr, "%s : lock %d at %d\n", __func__, error, line);
    return false;
  }
  line_ = line;
  return true;
}

bool EthercatOobCom::unlock(unsigned line)
{
  int error;
  if (0 != (error = pthread_mutex_unlock(&mutex_)))
  {
    fprintf(stderr, "%s : unlock %d at %d\n", __func__, error, line);
    return false;
  }
  line_ = 0;
  return true;
}

// OOB replacement for netif->txandrx()
// Returns true for success, false for dropped packet

bool EthercatOobCom::txandrx_once(struct EtherCAT_Frame * frame)
{
  assert(frame != NULL);

  if (!lock(__LINE__))
    return false;

  // Wait for an opening to send frame
  while (state_ != IDLE)
  {
    pthread_cond_wait(&share_cond_, &mutex_);
  }
  frame_ = frame;
  state_ = READY_TO_SEND;

  // RT control loop will send frame
  do
  {
    pthread_cond_wait(&busy_cond_, &mutex_);
  }
  while (state_ != WAITING_TO_RECV);

  // Packet has been sent, wait for recv
  bool success = false;
  if (handle_ >= 0)
    success = ni_->rx(frame_, ni_, handle_);
  handle_ = -1;

  // Allow other threads to send data
  assert(frame_ == frame);
  state_ = IDLE;
  pthread_cond_signal(&share_cond_);

  unlock(__LINE__);

  return success;
}

bool EthercatOobCom::txandrx(struct EtherCAT_Frame * frame)
{
  static const unsigned MAX_TRIES = 10;
  for (unsigned tries = 0; tries < MAX_TRIES; ++tries)
  {
    if (this->txandrx_once(frame))
      return true;
  }
  return false;
}

// Called by RT control loop to send oob data

void EthercatOobCom::tx()
{
  if (!trylock(__LINE__))
    return;

  if (state_ == READY_TO_SEND)
  {
    // Packet is in need of being sent
    assert(frame_ != NULL);
    handle_ = ni_->tx(frame_, ni_);
    state_ = WAITING_TO_RECV;
    pthread_cond_signal(&busy_cond_);
  }

  unlock(__LINE__);
}
