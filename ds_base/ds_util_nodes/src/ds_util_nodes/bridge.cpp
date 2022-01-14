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

#include <cassert>

#include "ds_util_nodes/bridge.h"


namespace ds_util_nodes {

Bridge::Bridge()
    : DsProcess()
{
}

Bridge::Bridge(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

Bridge::~Bridge() = default;

void
Bridge::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_conn = addConnection(
    "connection",
    boost::bind(&Bridge::_on_in_msg, this, _1)
  );
  m_conn->setRawPublisherEnable(false);
}

void
Bridge::setupSubscriptions()
{
  ds_base::DsProcess::setupSubscriptions();

  m_sub = nodeHandle().subscribe(
    ros::this_node::getName() + "/out",
    512,
    &Bridge::_on_out_msg, this
  );
}

void
Bridge::setupPublishers()
{
  ds_base::DsProcess::setupPublishers();
  m_pub = nodeHandle().advertise<ds_core_msgs::RawData>(
    ros::this_node::getName() + "/in",
    512
  );
}

void
Bridge::_on_in_msg(const ds_core_msgs::RawData& msg)
{
  m_pub.publish(msg);
}

void
Bridge::_on_out_msg(const ds_core_msgs::RawData& msg)
{
  std::stringstream out_msg;
  for (int i = 0; i < msg.data.size(); i++){
    out_msg << msg.data[i];
  }
  m_conn->send(out_msg.str());
}

} //namespace
