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

//
// Created by jvaccaro on 1/21/20.
//
#include "ds_util_nodes/responder.h"

namespace ds_util_nodes {

Responder::Responder()
    : DsProcess()
{
}

Responder::Responder(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

Responder::~Responder()
{
  m_fstream.close();
}

void
Responder::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_conn = addConnection("connection", boost::bind(&Responder::_on_msg, this, _1));
  auto file = ros::param::param<std::string>("~input_file", "/home/sentry/input.txt");
  m_fstream.open(file);
  m_newline = ros::param::param<bool>("~newline", false);
}

void
Responder::_on_msg(const ds_core_msgs::RawData& msg)
{
  auto next_message = _read_next_message();
  if (m_newline){
    m_conn->send(next_message + "\r\n");
  } else {
    m_conn->send(next_message);
  }
}

std::string
Responder::_read_next_message()
{
  std::string next_message;
  if (std::getline(m_fstream, next_message))
  {
    return next_message;
  } else {
    return "File ended!";
  }

}

} //namespace