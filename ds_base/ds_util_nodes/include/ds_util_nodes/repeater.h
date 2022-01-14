//
// Created by jvaccaro on 7/10/19.
//

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

#ifndef PROJECT_REPEATER_H
#define PROJECT_REPEATER_H
#include <ds_base/ds_process.h>
#include <ds_asio/ds_connection.h>
#include <ds_core_msgs/RawData.h>

namespace ds_util_nodes{

class Repeater : public ds_base::DsProcess {
  /// The purpose of this node is to repeat any incoming over messages
  /// from one ds_connection onto the other ds_connection.
  /// Can be one-way (default) or bidirectional, with incoming messages
  /// sent as outgoing messages on the opposite connection.
  /// params
  /// connection_1 : (ds_connection) defines incoming message connection
  /// connection_2 : (ds_connection) defines outgoing message connection
  /// bidirectional : (bool) if true, then incoming messages on
  ///                 connection_2 get sent as outgoing messages on
  ///                 connection_1
  DS_DISABLE_COPY(Repeater)

 public:
  Repeater();
  Repeater(int argc, char* argv[], const std::string& name);
  ~Repeater() override;

  void _on_msg_1(const ds_core_msgs::RawData& raw);
  void _on_msg_2(const ds_core_msgs::RawData& raw);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_conn_1;
  boost::shared_ptr<ds_asio::DsConnection> m_conn_2;
  bool m_is_bidirectional;
};

}
#endif //PROJECT_REPEATER_H
