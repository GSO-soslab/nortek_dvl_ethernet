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

#ifndef PROJECT_DS_BUS_DEVICE_PRIVATE_H
#define PROJECT_DS_BUS_DEVICE_PRIVATE_H

#include "ds_base/ds_bus_device.h"
#include "ds_base/ds_process.h"
#include <ds_base/util.h>

#include <ds_core_msgs/Status.h>
#include <ds_core_msgs/RawData.h>
#include <ds_core_msgs/IoSMcommand.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

namespace ds_base
{
struct DsBusDevicePrivate
{
  DsBusDevicePrivate();

  virtual ~DsBusDevicePrivate() = default;

  // disable copy operations
  DsBusDevicePrivate(const DsBusDevicePrivate&) = delete;
  void operator=(const DsBusDevicePrivate&) = delete;

  /// @brief Convenience method to send an I/O state machine command and get the reply
  ds_core_msgs::IoSMcommand::Response sendIosmCommand(const ds_core_msgs::IoSMcommand::Request& cmd);

  boost::uuids::uuid uuid_;

  std::unordered_map<std::string, ros::Publisher> publishers_;         //!< Collection of data message publishers
  std::unordered_map<std::string, boost::any> last_message_;           //!< Last message published on given topic
  std::unordered_map<std::string, ros::Time> last_message_timestamp_;  //!< Last message timestamp.

  ros::Duration message_timeout_;  //!< Time between valid messages to consider bad.

  /// \brief The name of the node that's managing this bus
  std::string bus_node_name_;

  /// \brief The topic that spits all bus traffic
  ros::Subscriber bus_;

  /// \brief Topic to send preempt commands to the bus
  ros::Publisher preempt_cmd_;

  /// \brief Topic to update commands at a high rate
  ros::Publisher update_cmd_;

  /// \brief The service that controls the I/O state machine on that bus
  ros::ServiceClient iosm_cmd_;
};
}  // namespace ds_base

#endif  // PROJECT_DS_BUS_DEVICE_PRIVATE_H
