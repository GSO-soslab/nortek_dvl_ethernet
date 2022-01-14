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
#include "ds_util_nodes/logger.h"
#include <fstream>

namespace ds_util_nodes {

Logger::Logger()
    : DsProcess()
{
}

Logger::Logger(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

Logger::~Logger() = default;

void
Logger::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_conn = addConnection("connection", boost::bind(&Logger::_on_msg, this, _1));
  m_filename = ros::param::param<std::string>("~filename", "/home/jvaccaro/default_filename.dat");
  m_is_binary = ros::param::param<bool>("~binary", false);
  m_is_timestamped = ros::param::param<bool>("~timestamped", false);
  if (m_is_binary && m_is_timestamped){
    ROS_ERROR_STREAM("Binary data cannot be timestamped! Proceeding to log binary data without timestamp");
  }
  m_newline = ros::param::param<bool>("~newline", false);
  if (m_is_binary && m_newline){
    ROS_ERROR_STREAM("Binary data will not log with skipped lines! Proceeding to log binary data without newline");
  }
}

void
Logger::_on_msg(const ds_core_msgs::RawData& msg)
{
  if (m_is_binary){
    log_binary(msg);
  } else {
    log_ascii(msg);
  }
}

void
Logger::log_binary(const ds_core_msgs::RawData& msg)
{
  auto fs = new std::ofstream();
  fs->open (m_filename, std::ofstream::app | std::ios::binary);
  auto size = msg.data.size();
  auto data = reinterpret_cast<const char*>(msg.data.data());
  fs->write(data, size);
  fs->close();
}

void
Logger::log_ascii(const ds_core_msgs::RawData& msg)
{
  std::ofstream fs;
  fs.open (m_filename, std::ios::app);
//  if (m_is_timestamped){
//    auto facet = new boost::posix_time::time_facet("%Y%m%d %H%M ");
//    fs.imbue(std::locale(fs.getloc(), facet));
//    fs << boost::posix_time::second_clock::universal_time();
//  }
  fs << std::string{ reinterpret_cast<const char*>(msg.data.data()), msg.data.size() };;
  if (m_newline){
    fs << "\r\n";
  }
  fs.close();
}

} //namespace