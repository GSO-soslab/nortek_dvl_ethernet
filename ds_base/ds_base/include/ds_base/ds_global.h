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
// Created by zac on 1/25/18.
//

#ifndef DS_BASE_DS_GLOBAL_H
#define DS_BASE_DS_GLOBAL_H

namespace ds_base
{
#define DS_DISABLE_COPY(Class)                                                                                         \
  Class(const Class&) = delete;                                                                                        \
  Class& operator=(const Class&) = delete;

#define DS_DECLARE_PRIVATE(Class)                                                                                      \
  inline Class##Private* d_func()                                                                                      \
  {                                                                                                                    \
    return reinterpret_cast<Class##Private*>(d_ptr_.get());                                                            \
  }                                                                                                                    \
  inline const Class##Private* d_func() const                                                                          \
  {                                                                                                                    \
    return reinterpret_cast<const Class##Private*>(d_ptr_.get());                                                      \
  }                                                                                                                    \
  friend class Class##Private;

#define DS_DECLARE_PRIVATE_D(Dptr, Class)                                                                              \
  inline Class##Private* d_func()                                                                                      \
  {                                                                                                                    \
    return reinterpret_cast<Class##Private*>(Dptr);                                                                    \
  }                                                                                                                    \
  inline const Class##Private* d_func() const                                                                          \
  {                                                                                                                    \
    return reinterpret_cast<const Class##Private*>(Dptr);                                                              \
  }                                                                                                                    \
  friend class Class##Private;

#define DS_DECLARE_PUBLIC(Class)                                                                                       \
  inline Class* q_func()                                                                                               \
  {                                                                                                                    \
    return static_cast<Class*>(q_ptr_);                                                                                \
  }                                                                                                                    \
  inline const Class* q_func() const                                                                                   \
  {                                                                                                                    \
    return static_cast<const Class*>(q_ptr_);                                                                          \
  }                                                                                                                    \
  friend class Class;

#define DS_D(Class) Class##Private* const d = d_func()
#define DS_Q(Class) Class* const q = q_func()
}

#endif  // DS_BASE_DS_GLOBAL_H
