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

#ifndef PROJECT_LOGGER_H
#define PROJECT_LOGGER_H
#include <ds_base/ds_process.h>
#include <ds_asio/ds_connection.h>
#include <ds_core_msgs/RawData.h>

namespace ds_util_nodes{

class Logger : public ds_base::DsProcess {
  /// The purpose of this node is to log any incoming ds_connection
  /// message to an ascii or binary file.
  /// params: connection (ds_connection) : Defines connection for incoming msg
  ///         filename (string) : Path and file for logging. Appends to non-empty files.
  ///         binary (bool) : Log message as binary data directly to file.
  ///         timestamped (bool) : Add a timestamp before ascii messages (on the same line)
  ///         newline (bool) : Add a newline after ascii messages
  DS_DISABLE_COPY(Logger)

 public:
  Logger();
  Logger(int argc, char* argv[], const std::string& name);
  ~Logger() override;

  void _on_msg(const ds_core_msgs::RawData& raw);
  void log_ascii(const ds_core_msgs::RawData& raw);
  void log_binary(const ds_core_msgs::RawData& raw);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_conn;
  bool m_is_binary;
  bool m_is_timestamped;
  bool m_newline;
  std::string m_filename;
  int m_filecount;
};

}
#endif //PROJECT_Logger_H
