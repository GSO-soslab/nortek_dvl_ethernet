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

#ifndef PROJECT_SPITTER_H
#define PROJECT_SPITTER_H
#include <ds_base/ds_process.h>
#include <ds_asio/ds_connection.h>
#include <ds_core_msgs/RawData.h>

namespace ds_util_nodes{

class Spitter : public ds_base::DsProcess {
  /// The purpose of this node is to spit a single string message
  /// along a ds_connection at a given interval.
  /// params
  ///        connection : (ds_connection) defines outgoing connection
  ///        message : (string) defines the outgoing message
  ///        interval_msec : (int) defines time interval in ms between msgs
  ///        newline : (bool) if true, appends "\r\n" to the end of a message.
  DS_DISABLE_COPY(Spitter)

 public:
  Spitter();
  Spitter(int argc, char* argv[], const std::string& name);
  ~Spitter() override;

  void _on_msg(const ds_core_msgs::RawData& raw);
  void _on_timeout(const ros::TimerEvent&);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_conn;
  int m_interval_msec;
  bool m_newline;
  ros::Timer m_timer;
  std::string m_message;
};

}
#endif //PROJECT_SPITTER_H
