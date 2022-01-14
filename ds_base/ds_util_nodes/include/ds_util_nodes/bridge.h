//
// Created by rgovostes on 4/16/20.
//

/**
* Copyright 2020 Woods Hole Oceanographic Institution
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

#ifndef PROJECT_BRIDGE_H
#define PROJECT_BRIDGE_H

#include <ros/ros.h>

#include <ds_base/ds_process.h>
#include <ds_asio/ds_connection.h>
#include <ds_core_msgs/RawData.h>

namespace ds_util_nodes{

class Bridge : public ds_base::DsProcess {
  /// The purpose of this node is to publish any incoming messages from a
  /// ds_connection on a given topic. Messages published to another topic
  /// can be written out to same connection.
  ///
  /// params
  /// connection : (ds_connection) defines message connection
  /// inbound_topic : (string) topic to publish inbound messages on
  /// output_topic : (string) topic to publish outbound messages on
  DS_DISABLE_COPY(Bridge)

 public:
  Bridge();
  Bridge(int argc, char* argv[], const std::string& name);
  ~Bridge() override;

 protected:
  void setupConnections() override;
  void setupSubscriptions() override;
  void setupPublishers() override;

  void _on_in_msg(const ds_core_msgs::RawData& raw);
  void _on_out_msg(const ds_core_msgs::RawData& raw);

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_conn;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
};

}

#endif //PROJECT_BRIDGE_H
