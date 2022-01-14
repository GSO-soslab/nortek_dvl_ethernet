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
#ifndef DS_SENSOR_SENSOR_BASE_PRIVATE_H
#define DS_SENSOR_SENSOR_BASE_PRIVATE_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/Status.h"

#include <boost/any.hpp>
#include <boost/uuid/uuid.hpp>

#include <unordered_map>

namespace ds_base
{
/// @brief Private implmentation for SensorBase class
///
/// This structure hides the actual implementation details for classes based on SensorBase.
struct SensorBasePrivate
{
  SensorBasePrivate() = default;
  virtual ~SensorBasePrivate() = default;

  DS_DISABLE_COPY(SensorBasePrivate)

  ros::Duration message_timeout_;  //!< Acceptable duration between sensor messages.

  /// @brief A string identifying the instrument frame for this sensor in TF
  std::string frame_id_;

  ///@brief last timestamps, checked by SensorBase::checkTimestamps
  SensorBase::TimestampMap last_timestamps_;

  ///@brief Connections available to the SensorBase::sendCommand method
  SensorBase::ConnectionMap connections_;
  ros::ServiceServer send_command_service_;  //!< ros::Service for sending commands
};
}

#endif  // DS_SENSOR_SENSOR_BASE_PRIVATE_H
