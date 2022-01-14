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

#include "ds_param_private.h"
#include "ds_param_conn_private.h"

#include <sstream>
#include <stdexcept>

namespace ds_param
{
ParamConnection::ParamConnection(ros::NodeHandle& _h) : impl(std::make_shared<ParamConnectionPrivate>(_h))
{
}

ParamConnection::~ParamConnection()
{
  impl->updatePub.shutdown();
  impl->updateSub.shutdown();
  impl->descriptionPub.shutdown();
}

std::shared_ptr<ParamConnection> ParamConnection::create(ros::NodeHandle& handle)
{
  return std::shared_ptr<ParamConnection>(new ParamConnection(handle));
}

template <typename T>
typename T::Ptr ParamConnection::connect(const std::string& param_name, bool advertise)
{
  std::string full_name = impl->handle.resolveName(param_name);
  typename T::Ptr ret;
  auto iter = impl->params.find(full_name);
  if (iter != impl->params.end())
  {
    ROS_INFO_STREAM("Variable named \"" << full_name << "\" (given \"" << param_name << "\") already exists!");
    ret = std::dynamic_pointer_cast<T>(iter->second);
    if (!ret)
    {
      std::stringstream msg;
      msg << "Variable named \"" << full_name << "\" already exists, but could not be cast to the requested type";
      ROS_ERROR_STREAM(msg.str());
      throw std::invalid_argument(msg.str());
    }
    else
    {
      if (advertise)
      {
        // FORCE us to advertise an existing variable if connection asks us to, whether
        // the existing version does or not
        ret->setAdvertise(true);
        impl->publishDescription();
      }
    }
    return ret;
  }

  if (!impl->handle.hasParam(full_name))
  {
    std::stringstream msg;
    msg << "Variable named \"" << full_name << "\" does not exist on the parameter server!";
    ROS_ERROR_STREAM(msg.str());
    throw std::invalid_argument(msg.str());
  }

  ret = std::make_shared<T>(impl, full_name, advertise);

  ret->loadFromServer();

  // add to our global list of parameters
  impl->params.insert(std::make_pair(full_name, std::static_pointer_cast<UpdatingParam>(ret)));

  // update our description
  impl->publishDescription();

  return ret;
}

const std::string& ParamConnection::connName() const
{
  return impl->conn_name;
};

void ParamConnection::setCallback(const Callback_t& _cb)
{
  impl->callback = _cb;
}

void ParamConnection::lock()
{
  impl->lock();
}

void ParamConnection::unlock()
{
  impl->unlock();
}

/// \brief Check if this object is locked
bool ParamConnection::IsLocked() const
{
  return impl->IsLocked();
}

// explicit instantiations-- this basically fills in the template during the build so that
// the necessary implementation ends up in hte library
template typename BoolParam::Ptr ParamConnection::connect<BoolParam>(const std::string& param_name, bool advertise);
template typename IntParam::Ptr ParamConnection::connect<IntParam>(const std::string& param_name, bool advertise);
template typename FloatParam::Ptr ParamConnection::connect<FloatParam>(const std::string& param_name, bool advertise);
template typename DoubleParam::Ptr ParamConnection::connect<DoubleParam>(const std::string& param_name, bool advertise);
template typename StringParam::Ptr ParamConnection::connect<StringParam>(const std::string& param_name, bool advertise);
template typename EnumParam::Ptr ParamConnection::connect<EnumParam>(const std::string& param_name, bool advertise);
}
