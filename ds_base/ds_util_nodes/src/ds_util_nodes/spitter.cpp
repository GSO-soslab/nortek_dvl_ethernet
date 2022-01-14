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
#include "ds_util_nodes/spitter.h"

namespace ds_util_nodes {

Spitter::Spitter()
    : DsProcess()
{
}

Spitter::Spitter(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

Spitter::~Spitter()
{
  m_timer.stop();
}

void
Spitter::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_conn = addConnection("connection", boost::bind(&Spitter::_on_msg, this, _1));
  m_interval_msec = ros::param::param<int>("~interval_msec", 1000);
  m_newline = ros::param::param<bool>("~newline", false);
  m_message = ros::param::param<std::string>("~message", "DEFAULT_MSG ");
  m_timer = nodeHandle().createTimer(ros::Duration(m_interval_msec/1.0e3),
                                     &Spitter::_on_timeout, this);
  m_timer.start();

}

void
Spitter::_on_msg(const ds_core_msgs::RawData& msg)
{
}

void
Spitter::_on_timeout(const ros::TimerEvent&)
{
  if (m_newline){
    m_conn->send(m_message + "\r\n");
  } else {
    m_conn->send(m_message);
  }
}

} //namespace