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
#ifndef DS_SERIAL_H
#define DS_SERIAL_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <linux/serial.h>
#include <sys/ioctl.h>

namespace ds_asio
{
class DsSerial : public DsConnection
{
public:
  DsSerial(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh);

  void receive(void) override;

  void send(boost::shared_ptr<std::string> message) override;

  void setup(ros::NodeHandle& nh) override;

  boost::asio::serial_port& get_io_object(void);

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;
  void set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction);

  struct serial_rs485 rs485conf={0};;
  bool rs485Enable;
  
private:
  /// @brief Set the underlying serial port in "raw" mode
  ///
  /// This is called after opening the serial port, but before
  /// applying settings from the parameter server.
  ///
  /// Raw mode is taken as:
  /// settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  /// settings.c_oflag &= ~OPOST;
  /// settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  /// settings.c_cflag &= ~(CSIZE | PARENB);
  /// settings.c_cflag |= CS8;
  /// settings.c_cc[VTIME] = 0;
  /// settings.c_cc[VMIN] = 1;
  ///
  /// \param fd   File descriptor of the serial port
  void setRawMode(int fd);

  void setRS485Mode(int fd, bool rs485Enable);

  void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred);

  void handle_write(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                    std::size_t bytes_transferred);

  boost::asio::serial_port* port_;
  boost::array<char, 1> recv_buffer_;
  // char eol_;
  std::string eol_;  // end of line, may be a single character
  boost::asio::streambuf streambuf_;
  boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction_;
  uint8_t num_read_error_;
  ros::Timer read_error_retry_timer_;
  std::string port_name_;

private:
  // make noncopyable
  DsSerial(const DsSerial& other) = delete;
  DsSerial& operator=(const DsSerial& other) = delete;
};
}

#endif
