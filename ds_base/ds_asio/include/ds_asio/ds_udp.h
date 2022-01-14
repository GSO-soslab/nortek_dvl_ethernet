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
#ifndef DS_UDP_H
#define DS_UDP_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"

#include <vector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>

using boost::asio::ip::udp;

namespace ds_asio
{
class DsUdp : public DsConnection
{
public:
  DsUdp(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh);

  void receive(void) override;

  void send(boost::shared_ptr<std::string> message) override;

  void setup(ros::NodeHandle& nh) override;

  udp::socket& get_io_object(void);

private:
  // make noncopyable
  DsUdp(const DsUdp& other) = delete;
  DsUdp& operator=(const DsUdp& other) = delete;

private:
  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

  void handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                   std::size_t bytes_transferred);

  std::unique_ptr<udp::socket> socket_;
  udp::endpoint* remote_endpoint_;
  std::vector<char> recv_buffer_; // default size is 512, but can get reset by param server
  uint8_t num_read_error_;
  ros::Timer read_error_retry_timer_;
};
}
#endif
