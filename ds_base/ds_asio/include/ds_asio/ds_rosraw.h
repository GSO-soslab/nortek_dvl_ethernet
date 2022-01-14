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
//
// Created by ivaughn on 8/30/19.
//

#ifndef DS_ROSRAW_H
#define DS_ROSRAW_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace ds_asio
{

/// \brief The DsRosRaw connection allows drivers to be tested by connecting to a previously-recorded
/// ds_core_msgs::RawData topic and playing it back through the DsConnection interface
class DsRosRaw : public DsConnection
{
 public:
  DsRosRaw(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh);

  void receive(void) override;

  void send(boost::shared_ptr<std::string> message) override;

  void setup(ros::NodeHandle& nh) override;

 private:
  DsRosRaw(const DsRosRaw& other) = delete;
  DsRosRaw& operator=(const DsRosRaw& other) = delete;

 private:
  void handle_receive(ds_core_msgs::RawData msg);

  // no receive buffer is needed
  ros::Subscriber raw_sub_;
  ros::Publisher out_pub_;
};

} // namespace ds_asio

#endif //DS_ROSRAW_H
