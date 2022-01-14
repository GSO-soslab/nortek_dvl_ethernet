/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by ivaughn on 2/15/18.
//

#include "ds_param/ds_param_guard.h"
#include "ds_param/ds_param_conn.h"

#include <stdexcept>

namespace ds_param
{
ParamGuard::ParamGuard(const ParamConnection::Ptr& _c) : conn(_c)
{
  // don't lock an already-locked connection
  if (conn->IsLocked())
  {
    // throw exception
    // note that our destructor will NOT be called because we're throwing an exception
    // in the constructor so explicitly clean up our conn member
    conn.reset();

    // ALWAYS make a pretty error message!
    std::string msg("ds_param::ParamGuard attempted to lock a connection named " + conn->connName() +
                    " but it was already locked!");
    ROS_ERROR_STREAM(msg);
    throw std::invalid_argument(msg);
  }

  // ok, everything looks good, go ahead and lock the connection
  conn->lock();
}

ParamGuard::~ParamGuard()
{
  unlock();
}

void ParamGuard::unlock()
{
  conn->unlock();
}
}
