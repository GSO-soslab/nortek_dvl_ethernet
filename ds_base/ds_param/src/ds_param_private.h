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
// Created by ivaughn on 1/30/18.
//

#ifndef PROJECT_DS_PARAM_PRIVATE_H
#define PROJECT_DS_PARAM_PRIVATE_H

#include <ds_param/ds_param.h>
#include <ds_param/ds_param_conn.h>

namespace ds_param
{
// we need an abstract base class here
class UpdatingParamPrivate
{
  // make noncopyable
private:
  UpdatingParamPrivate(const UpdatingParamPrivate&) = delete;
  UpdatingParamPrivate& operator=(const UpdatingParamPrivate&) = delete;

public:
  UpdatingParamPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
    : conn(_c), name(_n), advertise_flag(_a), dirty(false)
  {
  }
  virtual ~UpdatingParamPrivate() = default;

  std::shared_ptr<ParamConnectionPrivate> conn;
  std::string name;
  bool advertise_flag;
  bool dirty;
};

template <typename T>
class UpdatingParamTPrivate : public UpdatingParamPrivate
{
  // make noncopyable
private:
  UpdatingParamTPrivate(const UpdatingParamTPrivate&) = delete;
  UpdatingParamTPrivate& operator=(const UpdatingParamTPrivate&) = delete;

public:
  typedef T ValueType;
  T value;
  boost::optional<T> prev_value;

  UpdatingParamTPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
    : UpdatingParamPrivate(_c, _n, _a)
  {
  }
};

class UpdatingParamEnumPrivate : public UpdatingParamTPrivate<int>
{
  // make noncopyable
private:
  UpdatingParamEnumPrivate(const UpdatingParamEnumPrivate&) = delete;
  UpdatingParamEnumPrivate& operator=(const UpdatingParamEnumPrivate&) = delete;

public:
  /// \brief A list of name/value pairs for our enum
  std::vector<std::pair<std::string, int> > namedValues;

  UpdatingParamEnumPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
    : UpdatingParamTPrivate<int>(_c, _n, _a)
  {
  }
};
}
#endif  // PROJECT_DS_PARAM_PRIVATE_H
