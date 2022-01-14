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
#include "ds_asio/ds_udp.h"

namespace ds_asio
{
DsUdp::DsUdp(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh)
  : DsConnection(io_service, name, callback)
{
  setup(myNh);
  receive();
}

void DsUdp::setup(ros::NodeHandle& nh)
{
  DsConnection::setup(nh);

  // Get the parameter information
  int buffer_size = 512;
  if (nh.hasParam(ros::this_node::getName() + "/" + name_ + "/buffer_size")){
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/buffer_size", buffer_size, 512);
    ROS_INFO_STREAM("Buffer size set to "<<buffer_size);
  }
  recv_buffer_.resize(buffer_size);

  int udp_rx;
  if (nh.hasParam(ros::this_node::getName() + "/" + name_ + "/udp_rx"))
  {
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/udp_rx", udp_rx, 44444);
    ROS_INFO_STREAM("udp_rx exists=" << udp_rx);
  }
  else
  {
    ROS_ERROR_STREAM(ros::this_node::getName() + ": udp_rx does not exist");
    // socket_ = new udp::socket(io_service_, udp::endpoint(udp::v4(), 44444));
  }

  int udp_tx;
  if (nh.hasParam(ros::this_node::getName() + "/" + name_ + "/udp_tx"))
  {
    nh.getParam(ros::this_node::getName() + "/" + name_ + "/udp_tx", udp_tx);
    ROS_INFO_STREAM("udp_tx exists, =" << udp_tx);
  }
  else
  {
    ROS_ERROR_STREAM(ros::this_node::getName() + ": udp_tx does not exist, default to port 50000");
    udp_tx = 50000;
  }

  std::string udp_address;
  if (nh.hasParam(ros::this_node::getName() + "/" + name_ + "/udp_address"))
  {
    nh.getParam(ros::this_node::getName() + "/" + name_ + "/udp_address", udp_address);
    ROS_INFO_STREAM("Using udp_address==\"" << udp_address << "\"");
  }
  else
  {
    ROS_ERROR_STREAM(ros::this_node::getName() + ": udp_address does not exist, default to 127.0.0.1");
    udp_address = "127.0.0.1";
  }

  // Create the socket and endpoint
  auto udp_ip = boost::asio::ip::address::from_string(udp_address);
  // Check for multicast
  if (!udp_ip.is_multicast()){
    // Create the socket
    socket_ = std::unique_ptr<udp::socket>(new udp::socket(io_service_, udp::endpoint(udp::v4(), udp_rx)));
    remote_endpoint_ = new udp::endpoint(udp_ip, udp_tx);
  } else {
    ROS_ERROR_STREAM("Multicast address "<<udp_address<<" receive only");
    // Create the socket
    socket_ = std::unique_ptr<udp::socket>(new udp::socket(io_service_));
    auto listen_address = boost::asio::ip::address::from_string("0.0.0.0");
    udp::endpoint listen_endpoint(listen_address, udp_rx);
    socket_->open(listen_endpoint.protocol());
    socket_->set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_->bind(listen_endpoint);

    socket_->set_option(boost::asio::ip::multicast::join_group(udp_ip));

    remote_endpoint_ = new udp::endpoint();
    remote_endpoint_->port(udp_tx);
  }
  // Check for broadcast addresses
  std::vector<std::string> split_add;
  boost::algorithm::split(split_add, udp_address, boost::is_any_of("."));
  if (split_add.size() > 0)
  {
    if (!split_add.back().compare("255"))
    {
      ROS_INFO_STREAM("Setting socket broadcast option");
      boost::asio::socket_base::broadcast option(true);
      socket_->set_option(option);
    }
  }

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh.advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/" + name_ + "/raw", 1);
}

void DsUdp::receive(void)
{
  recv_buffer_.assign(recv_buffer_.size(), 0);
//  boostwrite (socket, boost::asio::buffer(boost::asio::new_buffers->recv_buffer_));
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
                         boost::bind(&DsUdp::handle_receive, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}

void DsUdp::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    num_read_error_ = 0;

    // Store timestamp as soon as received
    raw_data_.ds_header.io_time = ros::Time::now();

    // ROS_INFO_STREAM("UDP received: " << recv_buffer_.data());
    raw_data_.data = std::vector<unsigned char>(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
    raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
    if (raw_publisher_enabled_) {
      raw_publisher_.publish(raw_data_);
    }
    if (!callback_.empty())
    {
      callback_(raw_data_);
    }
    receive();
    return;
  }
  num_read_error_++;
  ROS_ERROR_STREAM("Read error on socket: " << error << " " << error.message());
  ROS_ERROR("%d consecutive read errors on port %d", num_read_error_, socket_->local_endpoint().port());
  if (num_read_error_ <= 10)
  {
    ROS_WARN_STREAM("Retrying read in 0.1s");
    read_error_retry_timer_.stop();
    read_error_retry_timer_.start();
    return;
  }

  ROS_FATAL("Too many read errors on port %d", socket_->local_endpoint().port());
  ROS_FATAL_STREAM("Shutting down ros.");
  ros::shutdown();
  ros::waitForShutdown();
  exit(1);
}

void DsUdp::send(boost::shared_ptr<std::string> message)
{
  // ROS_INFO_STREAM("Scheduling UDP send");
  socket_->async_send_to(boost::asio::buffer(*message), *remote_endpoint_,  // remote_endpoint_,
                         boost::bind(&DsUdp::handle_send, this, message, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
  // ROS_INFO_STREAM("UDP send scheduled");
}

void DsUdp::handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                        std::size_t bytes_transferred)
{
  // Store timestamp as soon as received
  raw_data_.ds_header.io_time = ros::Time::now();

  // ROS_INFO_STREAM("UDP data sent");
  raw_data_.data = std::vector<unsigned char>(message->begin(), message->begin() + bytes_transferred);
  raw_data_.data_direction = ds_core_msgs::RawData::DATA_OUT;
  if (raw_publisher_enabled_) {
    raw_publisher_.publish(raw_data_);
  }
}

udp::socket& DsUdp::get_io_object(void)
{
  return *socket_;
}
}
