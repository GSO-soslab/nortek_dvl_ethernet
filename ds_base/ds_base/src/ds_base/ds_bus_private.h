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

#ifndef PROJECT_DS_IOSM_PROCESS_PRIVATE_H
#define PROJECT_DS_IOSM_PROCESS_PRIVATE_H

#include "ds_asio/ds_iosm.h"
#include "ds_base/ds_bus.h"
#include <ds_base/util.h>

#include <ds_core_msgs/Status.h>
#include <ds_core_msgs/RawData.h>
#include <ds_core_msgs/IoSMcommand.h>
#include <ds_core_msgs/IoCommandList.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

namespace ds_base
{
struct DsBusPrivate
{
  DsBusPrivate() : message_timeout_(ros::Duration(-1)), uuid_(boost::uuids::nil_uuid())
  {
    // do nothing (else)
  }

  ~DsBusPrivate() = default;

  bool _data_recv(const ds_core_msgs::RawData& bytes)
  {
    last_message_timestamp_ = bytes.header.stamp;
    bus_pub_.publish(bytes);
    return true; // ALWAYS accept data
  }

  bool _service_req(const ds_core_msgs::IoSMcommand::Request& req, ds_core_msgs::IoSMcommand::Response& resp)
  {
    bool retval = true;
    for (auto iter = req.commands.begin(); iter != req.commands.end(); iter++)
    {
      ds_asio::IoCommand cmd(*iter);

      switch (req.iosm_command)
      {
        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_REGULAR:
          resp.retval.push_back(iosm->addRegularCommand(cmd));
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_REGULAR:
          iosm->overwriteRegularCommand(cmd.getId(), cmd);
          resp.retval.push_back(cmd.getId());
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_REGULAR:
          iosm->deleteRegularCommand(cmd.getId());
          resp.retval.push_back(cmd.getId());
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_PREEMPT:
          iosm->addPreemptCommand(cmd);
          resp.retval.push_back(1);  // preempt commands don't have an ID
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_SHUTDOWN:
        case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_SHUTDOWN:
        case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_SHUTDOWN:
          // TODO: Privateement these
          ROS_ERROR_STREAM("NOT IMPLEMENTED: Could not add shutdown command " << cmd.getCommand());
          retval = false;
          break;
      }
    }
    return retval;
  }

  void _preempt_cmd(const ds_core_msgs::IoCommandList& cmdList)
  {
    for (auto iter = cmdList.cmds.begin(); iter != cmdList.cmds.end(); iter++)
    {
      ds_asio::IoCommand cmd(*iter);

      iosm->addPreemptCommand(cmd);
    }
  }

  void _update_cmd(const ds_core_msgs::IoCommandList& cmdList)
  {
    for (auto iter = cmdList.cmds.begin(); iter != cmdList.cmds.end(); iter++)
    {
      ds_asio::IoCommand cmd(*iter);

      if (!iosm->overwriteRegularCommand(cmd.getId(), cmd))
      {
        ROS_ERROR_STREAM("Unable to find I/O state machine command ID " << cmd.getId() << ", cmdstr = \""
                                                                        << cmd.getCommand() << "\"");
      }
    }
  }

  /// @brief Publisher for all incoming bus traffic
  ros::Publisher bus_pub_;

  /// @brief Service handler for command & control
  ros::ServiceServer cmd_serv_;

  /// @brief Subscription for rapid acceptance of preempt commands
  ros::Subscriber preempt_sub_;

  /// @brief Subscription for rapid updating of periodic commands
  ros::Subscriber update_sub_;

  /// @brief Our I/O state machine
  boost::shared_ptr<ds_asio::IoSM> iosm;

  /// @brief The bus UUID
  boost::uuids::uuid uuid_;

  /// @brief Timestamp of last data on the bus
  ros::Time last_message_timestamp_;

  /// @brief Time between valid messages to consider bad
  ros::Duration message_timeout_;
};

}  // namespace ds_base

#endif  // PROJECT_DS_IOSM_PROCESS_PRIVATE_H
