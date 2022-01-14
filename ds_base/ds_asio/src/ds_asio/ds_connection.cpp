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
#include "ds_asio/ds_connection.h"

namespace ds_asio
{
DsConnection::DsConnection(boost::asio::io_service& _io) : io_service_(_io), raw_publisher_enabled_(true)
{
}

DsConnection::DsConnection(boost::asio::io_service& _io, std::string name, const ReadCallback& callback)
  : io_service_(_io), name_(name), callback_(callback), raw_publisher_enabled_(true)
{
  // do nothing else
}

DsConnection::~DsConnection()
{
  ;
}

void DsConnection::setup(ros::NodeHandle& nh) {
  nh.param<bool>(ros::this_node::getName() + "/" + name_ + "/publish_raw", raw_publisher_enabled_, true);
  if (raw_publisher_enabled_) {
    ROS_INFO_STREAM("Publishing raw messages on raw topic");
  } else {
    ROS_INFO_STREAM("Raw I/O publishing DISABLED");
  }
}

void DsConnection::send(const std::string& message)
{
  // asio really needs a shared_ptr to the message; this is just a convenience
  // wrapper to do the copy into a shared_ptr buffer easier on users
  this->send(boost::shared_ptr<std::string>(new std::string(message)));
}

const std::string& DsConnection::getName() const
{
  return name_;
}

boost::asio::io_service& DsConnection::getIoService()
{
  return io_service_;
}

const ReadCallback& DsConnection::getCallback() const
{
  return callback_;
}

void DsConnection::setCallback(const ReadCallback& _cb)
{
  callback_ = _cb;
}

bool DsConnection::getRawPublisherEnabled() const {
  return raw_publisher_enabled_;
}

void DsConnection::setRawPublisherEnable(bool v) {
  raw_publisher_enabled_ = v;
}

}
