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
#include "ds_process_private.h"
#include "ds_base/ds_process.h"

#include "ds_core_msgs/Status.h"

#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ds_base
{
DsProcessPrivate::DsProcessPrivate()
  : asio_(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio))
  , uuid_(boost::uuids::nil_uuid())
  , is_setup_(false)
  , is_critical_(false)
{
}

void DsProcessPrivate::updateStatusCheckTimer(DsProcess* base, ros::Duration period)
{
  if (period == status_check_period_)
  {
    return;
  }

  // Stop pending triggers.
  status_check_timer_.stop();

  // Negative durations disable the timer
  if (period < ros::Duration(0))
  {
    ROS_INFO_STREAM("Disabling status check timer");
    status_check_period_ = ros::Duration(-1);
    return;
  }

  status_check_period_ = period;
  status_check_timer_ = base->nodeHandle().createTimer(status_check_period_, &DsProcess::checkProcessStatus, base);
  ROS_INFO_STREAM("Status check timer set to " << status_check_period_);
}

void DsProcessPrivate::updateCriticalProcessTimer(DsProcess* base, ros::Duration period)
{
  if (period == critical_check_period_)
  {
    return;
  }

  // Stop pending triggers.
  critical_check_timer_.stop();

  // Negative durations disable the timer
  if (period < ros::Duration(0))
  {
    ROS_INFO_STREAM("Disabling status check timer");
    critical_check_period_ = ros::Duration(-1);
    return;
  }

  critical_check_period_ = period;
  critical_check_timer_ =
      base->nodeHandle().createTimer(critical_check_period_, &DsProcess::onCriticalProcessTimer, base);
  ROS_INFO_STREAM("Critical process check timer set to " << critical_check_period_);
}
}
