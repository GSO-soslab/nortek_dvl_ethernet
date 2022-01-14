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
#include "ds_base/ds_process.h"
#include "ds_process_private.h"
#include "ds_core_msgs/Status.h"

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ds_base
{
DsProcess::DsProcess() : d_ptr_(std::unique_ptr<DsProcessPrivate>(new DsProcessPrivate))
{
  if (ros::isInitialized()) {
    d_ptr_->my_node_handle_ = boost::make_shared<ros::NodeHandle>();
  } else {
    ROS_WARN("DsProcess starting up, but ROS not yet initialized!");
  }
}

DsProcess::DsProcess(int argc, char** argv, const std::string& name)
  : d_ptr_(std::unique_ptr<DsProcessPrivate>(new DsProcessPrivate))
{
  ros::init(argc, argv, name);
  d_ptr_->my_node_handle_ = boost::make_shared<ros::NodeHandle>();
}

DsProcess::~DsProcess() = default;

ros::NodeHandle DsProcess::nodeHandle(const std::string& ns)
{
  DS_D(DsProcess);
  auto nh = ros::NodeHandle(ns);
  nh.setCallbackQueue(d->asio_->callbackQueue());

  return nh;
}

void DsProcess::run()
{
  DS_D(DsProcess);
  if (!d->is_setup_)
  {
    setup();
  }

  if (d->status_check_timer_.isValid())
  {
    d->status_check_timer_.start();
  }
  d->asio_->run();
  d->status_check_timer_.stop();
}

void DsProcess::setDescriptiveName(const std::string& name) noexcept
{
  ROS_INFO_STREAM("Setting descriptive name to: " << name);
  DS_D(DsProcess);
  d->descriptive_node_name_ = name;
}

std::string DsProcess::descriptiveName() const noexcept
{
  const DS_D(DsProcess);
  return d->descriptive_node_name_;
}

ros::Duration DsProcess::statusCheckPeriod() const noexcept
{
  const DS_D(DsProcess);
  return d->status_check_period_;
}

void DsProcess::setStatusCheckPeriod(ros::Duration period) noexcept
{
  DS_D(DsProcess);
  d->updateStatusCheckTimer(this, period);
}

ros::Duration DsProcess::criticalProcessPeriod() const noexcept
{
  const DS_D(DsProcess);
  return d->critical_check_period_;
}

void DsProcess::setCriticalProcessPeriod(ros::Duration period) noexcept
{
  DS_D(DsProcess);
  d->updateCriticalProcessTimer(this, period);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::addConnection(const std::string& name,
                                                                  const ds_asio::ReadCallback& callback)
{
  auto nh = nodeHandle();
  DS_D(DsProcess);
  return d->asio_->addConnection(name, callback, nh);
}

boost::shared_ptr<ds_asio::IoSM> DsProcess::addIoSM(const std::string& iosm_name, const std::string& conn_name,
                                                    const ds_asio::IoCommand::ReadCallback& callback)
{
  auto nh = nodeHandle();
  DS_D(DsProcess);
  return d->asio_->addIoSM(iosm_name, conn_name, callback, nh);
}

boost::uuids::uuid DsProcess::uuid() const noexcept
{
  const DS_D(DsProcess);
  return d->uuid_;
}

ds_asio::DsAsio* DsProcess::asio(void)
{
  const DS_D(DsProcess);
  return d->asio_->asio();
}

void DsProcess::setup()
{
  DS_D(DsProcess);
  if (d->is_setup_)
  {
    return;
  }

  setupParameters();
  setupConnections();
  setupSubscriptions();
  setupPublishers();
  setupTimers();
  setupServices();

  d->is_setup_ = true;
}

void DsProcess::setupParameters()
{
  const auto health_check_period = ros::param::param<double>("~health_check_period", -1.0);
  if (health_check_period > 0)
  {
    ROS_INFO_STREAM("Setting status updated period to " << health_check_period << " seconds.");
  }
  else
  {
    ROS_INFO_STREAM("Disabling periodic status checks.");
  }
  setStatusCheckPeriod(ros::Duration(health_check_period));
  const auto name_ = ros::param::param<std::string>("~descriptive_name", "NO_NAME_PROVIDED");
  setDescriptiveName(name_);

  DS_D(DsProcess);
  if (ros::param::has("~uuid"))
  {
    d->uuid_ = boost::uuids::string_generator()(ros::param::param<std::string>("~uuid", "0"));
  }
  else
  {
    ROS_WARN_STREAM("No UUID loaded from parameter node.  Using value: " << d->uuid_);
  }

  bool isCritical = ros::param::param<bool>("~critical", false);
  if (isCritical)
  {
    ROS_INFO_STREAM(ros::this_node::getName() << " is critical");
    const auto ttl = ros::param::param<int>("~critical_ttl", 60);
    d->ttl_ = ttl;
    d->is_critical_ = true;
    const auto critical_check_period = ros::param::param<double>("~critical_check_period", 5.0);
    if (critical_check_period > 0)
    {
      ROS_INFO_STREAM("Setting critical process ttl broadcast period to " << critical_check_period << " seconds.");
    }
    else
    {
      ROS_INFO_STREAM("Disabling critical process ttl broadcast.");
    }
    setCriticalProcessPeriod(ros::Duration(critical_check_period));
  }
  else
  {
    ROS_INFO_STREAM(ros::this_node::getName() << " is not critical");
  }
  return;
}

void DsProcess::setupPublishers()
{
  DS_D(DsProcess);
  d->status_publisher_ = nodeHandle().advertise<ds_core_msgs::Status>(ros::this_node::getName() + "/status", 10, false);
  // ROS_ERROR_STREAM("Is Critical: " << d->is_critical_ << " " << ros::this_node::getName());
  if (d->is_critical_ == true)
  {
    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM("Critical node namespace: " << ns);
    std::vector<std::string> splitns;
    boost::algorithm::split(splitns, ns, boost::is_any_of("/"));
    std::string basens;
    for (int i = 0; i < splitns.size(); ++i)
    {
      ROS_DEBUG_STREAM(splitns[i]);
      if (splitns[i] != "")
      {
        basens = splitns[i];
        break;
      }
    }
    if (splitns.size() > 0)  // There is a base (domain) namespace
    {
      d->critical_process_publisher_ =
          nodeHandle().advertise<ds_core_msgs::CriticalProcess>("/" + basens + "/critical_process", 10, false);
    }
    else
    {
      ROS_ERROR_STREAM("Couldn't find a base (domain) namespace to advertise critical_process topic");
    }
  }
}

void DsProcess::checkProcessStatus(const ros::TimerEvent& event)
{
  const auto status = statusMessage();
  publishStatus(status);
}

void DsProcess::onCriticalProcessTimer(const ros::TimerEvent& event)
{
  const auto msg = criticalProcessMessage();
  publishCriticalProcess(msg);
}

ds_core_msgs::Status DsProcess::statusMessage()
{
  const auto now = ros::Time::now();

  auto status = ds_core_msgs::Status{};
  DS_D(DsProcess);
  status.descriptive_name = d->descriptive_node_name_;

  status.ds_header.io_time = now;
  std::copy(std::begin(d->uuid_.data), std::end(d->uuid_.data), std::begin(status.ds_header.source_uuid));

  // Default to GOOD
  status.status = ds_core_msgs::Status::STATUS_GOOD;

  return status;
}

ds_core_msgs::CriticalProcess DsProcess::criticalProcessMessage()
{
  const auto now = ros::Time::now();

  auto msg = ds_core_msgs::CriticalProcess{};
  DS_D(DsProcess);

  msg.header.stamp = now;
  msg.ttl = d->ttl_;
  msg.nodename = ros::this_node::getName();
  // ROS_ERROR_STREAM(msg.nodename);

  return msg;
}

void DsProcess::publishStatus(const ds_core_msgs::Status& msg)
{
  DS_D(DsProcess);
  d->status_publisher_.publish(msg);
}

void DsProcess::publishCriticalProcess(const ds_core_msgs::CriticalProcess& msg)
{
  DS_D(DsProcess);
  d->critical_process_publisher_.publish(msg);
}

void DsProcess::setUuid(const boost::uuids::uuid& uuid) noexcept
{
  DS_D(DsProcess);
  d->uuid_ = uuid;
}

void DsProcess::setTtl(int ttl)
{
  DS_D(DsProcess);
  d->ttl_ = ttl;
}

int DsProcess::getTtl(void)
{
  DS_D(DsProcess);
  return d->ttl_;
}
}
