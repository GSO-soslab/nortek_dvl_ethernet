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
#include "ds_util_nodes/repeater.h"

namespace ds_util_nodes {

Repeater::Repeater()
    : DsProcess()
{
}

Repeater::Repeater(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

Repeater::~Repeater() = default;

void
Repeater::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_conn_1 = addConnection("connection_1", boost::bind(&Repeater::_on_msg_1, this, _1));
  m_conn_2 = addConnection("connection_2", boost::bind(&Repeater::_on_msg_2, this, _1));
  m_is_bidirectional = ros::param::param<bool>("~bidirectional", false);
}

void
Repeater::_on_msg_1(const ds_core_msgs::RawData& msg)
{
  std::stringstream out_msg;
  for (int i=0; i<msg.data.size(); i++){
    out_msg << msg.data[i];
  }
  m_conn_2->send(out_msg.str());
}

void
Repeater::_on_msg_2(const ds_core_msgs::RawData& msg)
{
  if (!m_is_bidirectional){
    // Don't publish anything for a one-way repeater
    return;
  }
  std::stringstream out_msg;
  for (int i=0; i<msg.data.size(); i++){
    out_msg << msg.data[i];
  }
  m_conn_1->send(out_msg.str());
}

} //namespace