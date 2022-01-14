//
// Created by jvaccaro on 7/24/19.
//

/**
* Copyright 2019 Woods Hole Oceanographic Institution
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

#include "ds_util_nodes/void_service.h"

namespace ds_util_nodes{

VoidService::VoidService()
    : ds_base::DsProcess()
{
}

VoidService::VoidService(int argc, char* argv[], const std::string& name)
    : ds_base::DsProcess(argc, argv, name)
{
}

VoidService::~VoidService() = default;

bool
VoidService::callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_ERROR_STREAM("Service called!");
  return true;
}

void
VoidService::setupServices()
{
  ds_base::DsProcess::setupServices();
  auto srv_address = ros::param::param<std::string>("~service", "/service");
  m_srv = nodeHandle().advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>
      (srv_address, boost::bind(&VoidService::callback, this, _1, _2));
}

}