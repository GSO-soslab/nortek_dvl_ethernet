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
// Created by ivaughn on 1/15/18.
//

#include "ds_base/ds_bus.h"
#include "ds_bus_private.h"

using namespace ds_base;

// See EXTENDING.md
// Our constructors use the protected constructor from `DsProcess`, providing our
// own version of the private implementation class.
//
// This newly constructed DsBus::Private gets implicitly upcast to DsProcess::Private
// when passed to DsProcess's constructor.
//
// NOTE:  Our public constructors just forward on to our protected versions.  If
// we end up needing to add logic inside the constructors we'll only have to add
// it in two places now (the protected versions) instead of all four.
// Public default constructor:  use our own protected anolog
DsBus::DsBus() : DsProcess(), d_ptr_(std::unique_ptr<DsBusPrivate>(new DsBusPrivate))
{
  // do nothing
}

// Another public->protected forwarding.
DsBus::DsBus(int argc, char* argv[], const std::string& name)
  : DsProcess(argc, argv, name), d_ptr_(std::unique_ptr<DsBusPrivate>(new DsBusPrivate))
{
  // do nothing
}

DsBus::~DsBus() = default;

void DsBus::setupConnections()
{
  ds_base::DsProcess::setupConnections();

  DS_D(DsBus);
  d->iosm = addIoSM("statemachine", "instrument", boost::bind(&DsBusPrivate::_data_recv, d, _1));
}

void DsBus::setupPublishers()
{
  // call the superclass
  ds_base::DsProcess::setupPublishers();

  DS_D(DsBus);
  // add our additional secret sauce
  auto nh = nodeHandle();
  d->bus_pub_ = nh.advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/bus", 10, false);

  // the whole queue is controlled by a service
  d->cmd_serv_ = nh.advertiseService<ds_core_msgs::IoSMcommand::Request, ds_core_msgs::IoSMcommand::Response>(
      ros::this_node::getName() + "/cmd", boost::bind(&DsBusPrivate::_service_req, d, _1, _2));
  d->preempt_sub_ = nh.subscribe(ros::this_node::getName() + "/preempt_cmd", 10, &DsBusPrivate::_preempt_cmd, d);
  d->update_sub_ = nh.subscribe(ros::this_node::getName() + "/update_cmd", 10, &DsBusPrivate::_update_cmd, d);
}

void DsBus::checkProcessStatus(const ros::TimerEvent& event)
{
  const auto now = ros::Time::now();

  auto status = ds_core_msgs::Status();
  DS_D(DsBus);
  status.descriptive_name = descriptiveName();

  status.ds_header.io_time = now;
  const auto uuid_ = uuid();
  std::copy(std::begin(uuid_.data), std::end(uuid_.data), std::begin(status.ds_header.source_uuid));

  if (d->message_timeout_ < ros::Duration(0) || now - d->last_message_timestamp_ > d->message_timeout_)
  {
    status.status = ds_core_msgs::Status::STATUS_GOOD;
  }
  else
  {
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  publishStatus(status);
}

void DsBus::setupParameters()
{
  ds_base::DsProcess::setupParameters();

  DS_D(DsBus);
  d->message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

  auto generated_uuid = ds_base::generateUuid("bus_node_" + descriptiveName());
}
