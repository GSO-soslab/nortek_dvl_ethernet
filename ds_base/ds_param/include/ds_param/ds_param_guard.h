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

#ifndef PROJECT_DS_PARAM_GUARD_H
#define PROJECT_DS_PARAM_GUARD_H

#include <memory>

#include "ds_param_conn.h"

namespace ds_param
{
/// \brief A class to block updates from going out during its lifetime.
///
/// Used to allow multiple updates to happen in a single message
/// You use this as follows:
///
///  {
///    ParamGuard lock(conn);
///    var_foo.Set(12);
///    var_bar.Set(16);
///  } // lock passes out of scope and automatically unlocks
///    // The connection will be informed and send the most recent value of
///    // any variables that have changed since the lock was placed in a single message
///
class ParamGuard
{
public:
  ParamGuard(const ParamConnection::Ptr& _c);

  /// \brief Destructor automatically unlocks the param connection
  virtual ~ParamGuard();

  /// \brief Explicit unlock function
  void unlock();

  // delete our pointer allocation operators so that users have to use this on the
  // stack.
private:
  static void* operator new(size_t) = delete;
  static void* operator new[](size_t) = delete;

protected:
  ParamConnection::Ptr conn;
};
}
#endif  // PROJECT_DS_PARAM_GUARD_H
