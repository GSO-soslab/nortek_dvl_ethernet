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
#include "ds_asio/ds_serial.h"
#include <cstdlib>
#include <termios.h>

namespace ds_asio
{
DsSerial::DsSerial(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback,
                   ros::NodeHandle& myNh)
  : DsConnection(io_service, name, callback), num_read_error_(0)
{
  setup(myNh);
  receive();

  // The read error retry timer just calls receive again.
  auto retry_timer_callback = [](DsSerial* base, const ros::TimerEvent& event) { base->receive(); };

  read_error_retry_timer_ =
      myNh.createTimer(ros::Duration(1), boost::bind<void>(retry_timer_callback, this, _1), true, false);

  ROS_ASSERT(read_error_retry_timer_.isValid());
}

void DsSerial::setup(ros::NodeHandle& nh)
{
  DsConnection::setup(nh);

  std::string port_name;
  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/port", port_name, "/dev/ttyUSB0");
  ROS_INFO_STREAM("Serial port: " << port_name);

  bool serial_rs485;
  nh.param<bool>(ros::this_node::getName() + "/" + name_ + "/serial_rs485", serial_rs485, false);
  if (serial_rs485)
  {
   ROS_WARN_STREAM("RS485 Mode Enabled, setting ports to RS485");
   rs485Enable = true;
  }
  else {
    rs485Enable = false;
  }
  
  int baud_rate;
  nh.param<int>(ros::this_node::getName() + "/" + name_ + "/baud", baud_rate, 9600);
  ROS_INFO_STREAM("Baud rate: " << baud_rate);

  int data_bits;
  nh.param<int>(ros::this_node::getName() + "/" + name_ + "/data_bits", data_bits, 8);
  ROS_INFO_STREAM("Data bits: " << data_bits);

  std::string myMatch;
  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/matcher", myMatch, "match_char");
  ROS_INFO_STREAM("Matcher: " << myMatch);
  if (!myMatch.compare("match_char"))
  {
    // char delimiter;
    std::string hexAscii;
    nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/delimiter", hexAscii, "0A");
    uint8_t delimiter;
    if (!sscanf(hexAscii.c_str(), "%hhX", &delimiter))
    {
      ROS_FATAL_STREAM("Unable to create delimiter from string: " << hexAscii);
      ROS_BREAK();
    }
    ROS_INFO("Hex ascii (dec): %d hex: %x", delimiter, delimiter);
    set_matcher(match_char(static_cast<char>(delimiter)));
  }
  else if (!myMatch.compare("match_header_length"))
  {
    int length;
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/length", length, 833);
    std::string hexAscii;
    nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/header", hexAscii, "7F7F");
    std::vector<unsigned char> myHeader;
    for (int i = 0; i < hexAscii.length(); i += 2)
    {
      std::string byteString = hexAscii.substr(i, 2);
      unsigned int myByte;
      sscanf(byteString.c_str(), "%X", &myByte);
      myHeader.push_back((unsigned char)myByte);
    }
    set_matcher(match_header_length(myHeader, length));
  }
  else if (!myMatch.compare("match_header_read_length"))
  {
    std::string hexAscii;
    nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/header", hexAscii, "7F7F");
    std::vector<unsigned char> myHeader;
    for (int i = 0; i < hexAscii.length(); i += 2)
    {
      std::string byteString = hexAscii.substr(i, 2);
      unsigned int myByte;
      sscanf(byteString.c_str(), "%X", &myByte);
      myHeader.push_back((unsigned char)myByte);
    }
    int length_location_bytes;
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/length_location_bytes", length_location_bytes, 2);
    int length_field_bytes;
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/length_field_bytes", length_field_bytes, 2);
    bool is_msb_first;
    nh.param<bool>(ros::this_node::getName() + "/" + name_ + "/is_msb_first", is_msb_first, true);
    int add_to_length;
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/add_to_length", add_to_length, 0);
    int max_length;
    nh.param<int>(ros::this_node::getName() + "/" + name_ + "/max_length", max_length, 512);

    set_matcher(match_header_read_length(myHeader, length_location_bytes, length_field_bytes, is_msb_first,
                                         add_to_length, max_length));
  }
  else if (!myMatch.compare("match_multi_header_length"))
  {
    std::vector<std::string> header_strs;
    std::vector<std::vector<unsigned char>> headers;
    std::vector<int> lengths;
    nh.param(ros::this_node::getName() + "/" + name_ + "/lengths", lengths, lengths);
    nh.param(ros::this_node::getName() + "/" + name_ + "/headers", header_strs, header_strs);

    for (const auto hdr_str : header_strs)
    {
      std::vector<unsigned char> hdr;
      for (int i = 0; i < hdr_str.length(); i += 2)
      {
        std::string byteString = hdr_str.substr(i, 2);
        unsigned int myByte;
        sscanf(byteString.c_str(), "%X", &myByte);
        hdr.push_back((unsigned char)myByte);
      }
      headers.push_back(hdr);
    }
    set_matcher(match_multi_header_length(headers, lengths));
  }
  else if (!myMatch.compare("match_header_pd0"))
  {
    set_matcher(match_header_pd0());
  }
  else if (!myMatch.compare("passthrough"))
  {
    set_matcher(passthrough());
  }

  std::string myParity;
  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/parity", myParity, "none");
  ROS_INFO_STREAM("Parity: " << myParity);

  int myStopbits;
  nh.param<int>(ros::this_node::getName() + "/" + name_ + "/stop_bits", myStopbits, 1);
  ROS_INFO_STREAM("Stop Bits: " << myStopbits);

  ROS_INFO_STREAM("Opening serial port in raw mode.");
  port_ = new boost::asio::serial_port(io_service_, port_name);
  setRawMode(port_->native_handle());
  setRS485Mode(port_->native_handle(), rs485Enable);
  port_name_ = port_name;

  ROS_DEBUG_STREAM("setting baud rate:  " << baud_rate);
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  ROS_DEBUG_STREAM("setting data bits:  " << data_bits);
  port_->set_option(boost::asio::serial_port_base::character_size(data_bits));

  if (myStopbits == 1)
  {
    ROS_DEBUG_STREAM("setting stop_bits: 1");
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  }
  else if (myStopbits == 2)
  {
    ROS_DEBUG_STREAM("setting stop_bits: 2");
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
  }
  else
  {
    ROS_FATAL_STREAM("Invalid stop_bits value: " << myStopbits);
    ROS_BREAK();
  }

  if (!myParity.compare("none"))
  {
    ROS_DEBUG_STREAM("setting parity: none");
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  }
  else if (!myParity.compare("odd"))
  {
    ROS_DEBUG_STREAM("setting parity: odd");
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
  }
  else if (!myParity.compare("even"))
  {
    ROS_DEBUG_STREAM("setting parity: even");
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
  }
  else
  {
    ROS_FATAL_STREAM("Invalid parity value: " << myParity);
    ROS_BREAK();
  }

  ROS_DEBUG_STREAM("Setting flow control: none");
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh.advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/" + name_ + "/raw", 1);
}

void DsSerial::receive(void)
{
  boost::asio::async_read_until(*port_, streambuf_, matchFunction_,
                                boost::bind(&DsSerial::handle_read, this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
}

void DsSerial::set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction)
{
  matchFunction_ = matchFunction;
}

void DsSerial::handle_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    num_read_error_ = 0;
    // Store timestamp as soon as received
    ros::Time now = ros::Time::now();
    raw_data_.ds_header.io_time = now;
    raw_data_.header.stamp = now;

    boost::asio::streambuf::const_buffers_type bufs = streambuf_.data();
    // ROS_DEBUG_STREAM("Streambuf data size: " << bytes_transferred);
    raw_data_.data = std::vector<unsigned char>(boost::asio::buffers_begin(bufs),
                                                boost::asio::buffers_begin(bufs) + bytes_transferred);
    // ROS_DEBUG_STREAM("Serial received: " << raw_data_.data.data());
    raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
    if (raw_publisher_enabled_) {
      raw_publisher_.publish(raw_data_);
    }
    if (!callback_.empty())
    {
      callback_(raw_data_);
    }
    raw_data_.data.clear();
    // The consume method of the streambuffer marks as used the bytes that we processed, so the next call to
    // async_read_until does not re-analyze them
    streambuf_.consume(bytes_transferred);
    receive();
    return;
  }
  num_read_error_++;
  ROS_ERROR_STREAM("Read error on port: " << error << " " << error.message());
  ROS_ERROR("%d consecutive read errors on port %s", num_read_error_, port_name_.c_str());
  if (num_read_error_ <= 10)
  {
    ROS_WARN_STREAM("Retrying read in 0.1s");
    read_error_retry_timer_.stop();
    read_error_retry_timer_.start();
    return;
  }

  ROS_FATAL_STREAM("Too many read errors on " << port_name_);
  ROS_FATAL_STREAM("Shutting down ros.");
  ros::shutdown();
  ros::waitForShutdown();
  exit(1);
}

void DsSerial::send(boost::shared_ptr<std::string> message)
{
  // ROS_INFO_STREAM("Scheduling serial send");
  boost::asio::async_write(*port_, boost::asio::buffer(*message),
                           boost::bind(&DsSerial::handle_write, this, message, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));

  // ROS_INFO_STREAM("Serial send scheduled");
}

void DsSerial::handle_write(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                            std::size_t bytes_transferred)
{
  // Store timestamp as soon as received
  raw_data_.ds_header.io_time = ros::Time::now();

  // ROS_INFO_STREAM("Serial data sent");
  raw_data_.data = std::vector<unsigned char>(message->begin(), message->begin() + bytes_transferred);
  raw_data_.data_direction = ds_core_msgs::RawData::DATA_OUT;
  if (raw_publisher_enabled_) {
    raw_publisher_.publish(raw_data_);
  }
  raw_data_.data.clear();
}

void DsSerial::setRawMode(int fd)
{
  struct termios settings;
  if (tcgetattr(fd, &settings))
  {
    ROS_ERROR_STREAM("Unable to get termios settings for fd: " << fd);
    return;
  }

  settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  settings.c_oflag &= ~OPOST;
  settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  settings.c_cflag &= ~(CSIZE | PARENB);
  settings.c_cflag |= CS8;
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN] = 1;

  if (tcsetattr(fd, TCSADRAIN, &settings))
  {
    ROS_ERROR_STREAM("Unable to set serial port to RAW mode");
  }
}

void DsSerial::setRS485Mode(int fd, bool rs485Enable)
{
  if (rs485Enable) {
    // DO SOME ERROR CHECKING
    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
      ROS_FATAL_STREAM("Error reading fd: "<<fd<<" with RS485 ENABLED ioctl");
      return;
    }
    
    rs485conf.flags |= SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | ~SER_RS485_RX_DURING_TX;

    //ROS_WARN_STREAM("RS485 ENABLED FLAGS SENT");

    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
      ROS_FATAL_STREAM("Error sending fd: "<<fd<<" with RS485 ENABLED ioctl");
      return;
    }
  }
  else {
    // DO SOME ERROR CHECKING
    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
      ROS_WARN_STREAM("Error reading fd: "<<fd<<" with RS485 DISABLED ioctl");
    }
    
    rs485conf.flags &= ~SER_RS485_ENABLED;

    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
      ROS_WARN_STREAM("Error sending fd: "<<fd<<" with RS485 DISABLED ioctl");
    }
  }
}

boost::asio::serial_port& DsSerial::get_io_object(void)
{
  return *port_;
}
}
