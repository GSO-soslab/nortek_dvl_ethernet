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
// Created by zac on 12/5/17.
//

#include "ds_base/sensor_base.h"
#include "sensor_base_private.h"
#include "ds_base/util.h"
#include "ds_base/StringCommand.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <iterator>

namespace ds_base
{
SensorBase::SensorBase() : DsProcess(), d_ptr_(std::unique_ptr<SensorBasePrivate>(new SensorBasePrivate))
{
}

SensorBase::SensorBase(int argc, char* argv[], const std::string& name)
  : DsProcess(argc, argv, name), d_ptr_(std::unique_ptr<SensorBasePrivate>(new SensorBasePrivate))
{
}

SensorBase::~SensorBase() = default;

void SensorBase::setTimeout(ros::Duration timeout) noexcept
{
  DS_D(SensorBase);
  if (timeout < ros::Duration(0))
  {
    ROS_INFO("Disabling message reception timeout check.");
    d->message_timeout_ = ros::Duration(-1);
  }
  else
  {
    d->message_timeout_ = timeout;
    ROS_INFO_STREAM("Message reception timeout: " << timeout);
  }
}

ros::Duration SensorBase::timeout() const noexcept
{
  const DS_D(SensorBase);
  return d->message_timeout_;
}

void SensorBase::setFrameId(const std::string& frame_id)
{
  DS_D(SensorBase);
  d->frame_id_ = frame_id;
}

const std::string& SensorBase::frameId() const noexcept
{
  const DS_D(SensorBase);
  return d->frame_id_;
}

void SensorBase::sendCommand(std::string command, std::string connection)
{
  DS_D(SensorBase);

  try
  {
    auto con = d->connections_.at(connection);
    auto msg = boost::shared_ptr<std::string>(new std::string);
    *msg = std::move(command);
    con->send(std::move(msg));
  }
  catch (std::out_of_range& e)
  {
    ROS_WARN_STREAM("No connection named '" << connection << "'");
  }
}

void SensorBase::sendCommand(std::string command, std::string connection, std::string suffix)
{
  if (!suffix.empty() && (!boost::algorithm::ends_with(command, suffix)))
  {
    command.append(suffix);
  }
  sendCommand(std::move(command), std::move(connection));
}

void SensorBase::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  DS_D(SensorBase);
  d->connections_["instrument"] = addConnection("instrument", boost::bind(&SensorBase::parseReceivedBytes, this, _1));
}

void SensorBase::setupServices()
{
  DsProcess::setupServices();

  // Need to implement the service callback as a non-capturing lambda (can't cast capturing lambdas
  // to boost::function types).  We'll provide the base pointer with boost::bind next
  auto callback = [](SensorBase* sensor_base, StringCommand::Request& request,
                     StringCommand::Response& response) -> bool {
    sensor_base->sendCommand(request.command, "instrument", "\r\n");
    return true;
  };

  auto nh = nodeHandle();

  DS_D(SensorBase);
  d->send_command_service_ = nh.advertiseService<StringCommand::Request, StringCommand::Response>(
      ros::this_node::getName() + "/send_command", boost::bind<bool>(callback, this, _1, _2));
}

void SensorBase::setupParameters()
{
  ds_base::DsProcess::setupParameters();
  DS_D(SensorBase);
  d->message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));
  d->frame_id_ = ros::param::param<std::string>("~frame_id", "base_link");

  auto serial_num = ros::param::param<std::string>("~serial_number", "0");
  auto generated_uuid = ds_base::generateUuid(serial_num);

  const auto provided_uuid = uuid();

  if (provided_uuid != generated_uuid)
  {
    // This really SHOULD be an error, but until we put the effort into making this a 
    // thing let's leave it an info
    ROS_INFO_STREAM("!!!POTENTIAL CONFIGURATION MISMATCH!!!");
    ROS_INFO_STREAM("Detected UUID mismatch.");
    ROS_INFO_STREAM("UUID (param server): " << provided_uuid);
    ROS_INFO_STREAM("UUID (generated): " << generated_uuid);
    ROS_INFO_STREAM("Using generated UUID!");
    setUuid(generated_uuid);
  }
  else
  {
    ROS_INFO_STREAM("UUID matches: " << provided_uuid);
  }
}

ds_asio::DsConnection* SensorBase::connection(const std::string& name)
{
  DS_D(SensorBase);
  try
  {
    return d->connections_.at(name).get();
  }
  catch (std::out_of_range& e)
  {
    ROS_WARN_STREAM("No connection named '" << name << "'");
    return nullptr;
  }
}

SensorBase::ConnectionMap& SensorBase::connections()
{
  DS_D(SensorBase);
  return d->connections_;
}

void SensorBase::updateTimestamp(std::string name)
{
  updateTimestamp(name, std::move(ros::Time::now()));
}

void SensorBase::updateTimestamp(std::string name, ros::Time time)
{
  DS_D(SensorBase);
  d->last_timestamps_[name] = std::move(time);
}

ros::Time SensorBase::lastTimestamp(const std::string& name) const noexcept
{
  const DS_D(SensorBase);
  try
  {
    return d->last_timestamps_.at(name);
  }
  catch (std::out_of_range& e)
  {
    return {};
  }
}

const SensorBase::TimestampMap& SensorBase::lastTimestamps() const noexcept
{
  const DS_D(SensorBase);
  return d->last_timestamps_;
}

void SensorBase::checkMessageTimestamps(ds_core_msgs::Status& status)
{
  const DS_D(SensorBase);

  // Message timeouts disabled.
  if (d->message_timeout_ < ros::Duration(0))
  {
    return;
  }

  // No timestamps?  That's an error.
  if (d->last_timestamps_.empty())
  {
    status.status = ds_core_msgs::Status::STATUS_ERROR;
    return;
  }

  auto it = std::begin(d->last_timestamps_);
  const auto now = ros::Time::now();

  while (it != std::end(d->last_timestamps_))
  {
    const auto age = now - it->second;
    if (age > d->message_timeout_)
    {
      ROS_WARN_STREAM("Last timestamp for " << it->first << "is " << it->second << " (" << age.toSec() << " seconds "
                                                                                                          "ago)");
      status.status = ds_core_msgs::Status::STATUS_WARN;
      return;
    }
    ++it;
  }
}

void SensorBase::checkProcessStatus(const ros::TimerEvent& event)
{
  auto msg = statusMessage();
  checkMessageTimestamps(msg);
  publishStatus(msg);
}

void SensorBase::fillHeaderMetadata(std_msgs::Header& hdr, ds_core_msgs::DsHeader& ds_hdr, const ros::Time& stamp,
                                    const ros::Time& io_time) const
{
  const DS_D(SensorBase);

  hdr.stamp = stamp;
  hdr.frame_id = d->frame_id_;

  ds_hdr.io_time = io_time;
  boost::uuids::uuid _uuid = uuid();  // get from parent class
  std::copy(std::begin(ds_hdr.source_uuid), std::end(ds_hdr.source_uuid), _uuid.begin());
}
}
