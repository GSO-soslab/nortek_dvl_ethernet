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
// Created by zac on 12/26/17.
//

#ifndef PROJECT_TESTDATASPITTER_H
#define PROJECT_TESTDATASPITTER_H

#include "ds_base/sensor_base.h"

class TestDataSpitter : public ds_base::SensorBase
{
protected:
  //  struct Impl;

public:
  explicit TestDataSpitter();
  TestDataSpitter(int argc, char* argv[], const std::string& name);

  ~TestDataSpitter() override;
#if 0
 private:

  /// @brief Access the underlying pimpl pointer.
  inline auto d_func() noexcept -> Impl*;

  /// @brief Access the underlying pimpl pointer.
  auto d_func() const noexcept -> Impl const*;

  /// @brief Protected constructor for subclasses using their own subclassed pimpl.
  explicit TestDataSpitter(std::unique_ptr<Impl> impl);
  explicit TestDataSpitter(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);
#endif
};

#endif  // PROJECT_TESTDATASPITTER_H
