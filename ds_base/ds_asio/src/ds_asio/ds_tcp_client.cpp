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
//
// Created by ivaughn on 8/30/19.
//

#include "ds_asio/ds_tcp_client.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::asio::ip;

namespace ds_asio
{

DsTcpClient::DsTcpClient(boost::asio::io_service& io_service, std::string name,
    const ReadCallback& callback, ros::NodeHandle& myNh) : DsConnection(io_service, name, callback), timeout_timer_(io_service) {
  error_count_ = 0;
  setup(myNh);
  connect();
}

void DsTcpClient::connect(void) {
  ROS_INFO_STREAM("Attempting to connect to " <<destination_.address() <<" on port " <<destination_.port());
  timeout_timer_.expires_from_now(timeout_period_);
  socket_->async_connect(destination_, boost::bind(&DsTcpClient::handle_connect, this, boost::asio::placeholders::error));
}

void DsTcpClient::receive(void) {

  timeout_timer_.expires_from_now(timeout_period_);
  //socket_->async_receive(boost::asio::buffer(recv_buffer_), 0, boost::bind(&DsTcpClient::handle_receive, this,
  //                                                                         boost::asio::placeholders::error,  boost::asio::placeholders::bytes_transferred));
  boost::asio::async_read_until(*socket_, streambuf_, matchFunction_,
          boost::bind(&DsTcpClient::handle_receive, this, boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  timeout_timer_.async_wait(boost::bind(&DsTcpClient::handle_timeout, this, boost::asio::placeholders::error));
}

void DsTcpClient::set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction) {
    matchFunction_ = matchFunction;
}
void DsTcpClient::send(boost::shared_ptr<std::string> message) {
  if (!socket_->is_open()) {
    ROS_ERROR_STREAM("Socket is not open yet!  Dropping outgoing message...");
    return;
  }
  ROS_INFO_STREAM("Scheduling TCP send!");
  socket_->async_send(boost::asio::buffer(*message), boost::bind(&DsTcpClient::handle_send,
                                                                 this, message, boost::asio::placeholders::error,
                                                                 boost::asio::placeholders::bytes_transferred));
}

void DsTcpClient::setup(ros::NodeHandle& nh) {
  DsConnection::setup(nh);

  double timeout;
  nh.param<double>(ros::this_node::getName() + "/" + name_ + "/timeout_sec", timeout, 30.0);
  long timeout_sec = static_cast<long>(timeout);
  long timeout_usec = static_cast<long>((timeout - timeout_sec)*1000000.0);

  timeout_period_ = boost::posix_time::seconds(timeout_sec) + boost::posix_time::microseconds(timeout_usec);

  ROS_INFO_STREAM("Resetting connections older than " <<boost::posix_time::to_simple_string(timeout_period_));

  std::string tcp_address;
  ROS_INFO_STREAM("Loading IP Address from " <<(ros::this_node::getName() + "/" + name_ + "/tcp_address"));
  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/tcp_address", tcp_address, "127.0.0.1");

  int tcp_port;
  nh.param<int>(ros::this_node::getName() + "/" + name_ + "/tcp_port", tcp_port, 20000);

  ROS_INFO_STREAM("Connecting to TCP server at address \"" <<tcp_address <<"\" on port \"" <<tcp_port <<"\"");

  auto host_addr = boost::asio::ip::address::from_string(tcp_address);
  destination_ = tcp::endpoint(host_addr, tcp_port);

  socket_.reset(new tcp::socket(io_service_));
  //Set default matcher function
  set_matcher(passthrough());
  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh.advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/" + name_ + "/raw", 1);
}

void DsTcpClient::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred) {
  if (error == boost::asio::error::operation_aborted) {
    // skip these
    return;
  }
  if (!error || error == boost::asio::error::message_size) {
    error_count_ = 0;

    // store timestamp as early as possible
    raw_data_.ds_header.io_time = ros::Time::now();

    // ROS_INFO_STREAM("TCP received: " << recv_buffer_.data());
    ROS_INFO_STREAM("Read " <<bytes_transferred <<" bytes!");
    boost::asio::streambuf::const_buffers_type bufs = streambuf_.data();
    raw_data_.data = std::vector<unsigned char>(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + bytes_transferred);
    //raw_data_.data = std::vector<unsigned char>(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
    raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
    if (raw_publisher_enabled_) {
      raw_publisher_.publish(raw_data_);
    }
    if (!callback_.empty())
    {
      callback_(raw_data_);
    }
    raw_data_.data.clear();
    streambuf_.consume(bytes_transferred);
    receive();
    return;
  }
  error_count_++;
  ROS_ERROR_STREAM("Read error on socket: " << error << " " << error.message());
  if (error_count_ <= 10)
  {
    ROS_WARN_STREAM("Attempting to reconnect in 10 seconds... error count=" <<static_cast<int>(error_count_));
    timeout_timer_.expires_from_now(boost::posix_time::seconds(10));
    timeout_timer_.async_wait(boost::bind(&DsTcpClient::handle_timeout, this, boost::asio::placeholders::error));
    return;
  }

  ROS_FATAL_STREAM("Too many read errors with TCP socket to " <<destination_.address() <<" port " <<destination_.port());
  ROS_FATAL_STREAM("Shutting down ros.");
  ROS_BREAK();
}

void DsTcpClient::handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                   std::size_t bytes_transferred) {
    // Store timestamp as soon as received
  raw_data_.ds_header.io_time = ros::Time::now();

  // ROS_INFO_STREAM("TCP data sent");
  raw_data_.data = std::vector<unsigned char>(message->begin(), message->begin() + bytes_transferred);
  raw_data_.data_direction = ds_core_msgs::RawData::DATA_OUT;
  if (raw_publisher_enabled_) {
    raw_publisher_.publish(raw_data_);
  }
}

void DsTcpClient::handle_connect(const boost::system::error_code& error) {
  if (error) {
    ROS_ERROR_STREAM("Error connecting TCP socket to " <<destination_.address()
                                                       <<" port " <<destination_.port()
                                                       <<": " <<error.message());
    error_count_++;
    if (error_count_ <= 10) {
      ROS_WARN_STREAM("Attempting to reconnect in 10 seconds... error count=" <<static_cast<int>(error_count_));
      timeout_timer_.expires_from_now(boost::posix_time::seconds(10));
      timeout_timer_.async_wait(boost::bind(&DsTcpClient::handle_timeout, this, boost::asio::placeholders::error));
    } else {
      ROS_FATAL("Unable to connect to destination!");
      ROS_BREAK();
    }
  }
  ROS_INFO_STREAM("Connected to " <<destination_.address() <<" port " <<destination_.port()
                                  <<", starting receive loop");
  receive();
}

void DsTcpClient::handle_timeout(const boost::system::error_code& error) {
  if (error == boost::asio::error::operation_aborted) {
    // timer was cancelled.  Ignore.
    return;
  }

  if (error) {
    ROS_ERROR_STREAM("Error in TCP timeout timer: " <<error.message());
    // just reconnect anyway.
  }

  ROS_WARN_STREAM("TCP Timeout detected talking to " <<destination_.address() <<" port " <<destination_.port()
  <<"; closing socket and re-connecting");
  socket_->close();

  connect();
}

} // namespace ds_asio
