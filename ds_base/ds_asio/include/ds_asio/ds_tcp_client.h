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

#ifndef DS_TCP_CLIENT_H
#define DS_TCP_CLIENT_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>

namespace ds_asio
{

/// \brief The DsTcpClient connection will connect to a TCP Server running on a possibly-remote host.  This
/// is especially useful with Moxas.
class DsTcpClient : public DsConnection
{
 public:
  DsTcpClient(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh);

  void receive(void) override;
  void connect(void); // kick off a connection attempt

  void send(boost::shared_ptr<std::string> message) override;

  void setup(ros::NodeHandle& nh) override;

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;
  void set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction);

 private:
  DsTcpClient(const DsTcpClient& other) = delete;
  DsTcpClient& operator=(const DsTcpClient& other) = delete;

 private:
  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

  void handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                   std::size_t bytes_transferred);

  void handle_connect(const boost::system::error_code& error);

  void handle_timeout(const boost::system::error_code& error);

  uint8_t error_count_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
  boost::asio::streambuf streambuf_;
  boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction_;
  boost::asio::ip::tcp::endpoint destination_;
  boost::asio::deadline_timer timeout_timer_;
  boost::posix_time::time_duration timeout_period_;
  double timeout_sec_;
  ros::Timer read_error_retry_timer_;
};

} // namespace ds_asio
#endif //DS_TCP_CLIENT_H
