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
#ifndef DS_CONNECTION_H
#define DS_CONNECTION_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_asio/ds_match_functions.h"

#include <ds_core_msgs/RawData.h>

#include <ros/ros.h>

namespace ds_asio
{
/// @typedef The typedef for read callbacks
typedef boost::function<void(ds_core_msgs::RawData)> ReadCallback;
// typedef boost::function <void(ds_core_msgs::RawDataConstPtr)> ReadCallback;

/// \class The base class for all connection types.
///
/// Connections mostly talk to I/O devices like serial ports or UDP ports or whatever.
class DsConnection
{
public:
  explicit DsConnection(boost::asio::io_service& _io);
  DsConnection(boost::asio::io_service& _io, std::string name, const ReadCallback& callback);
  virtual ~DsConnection();

  /// @brief An interface to send data through a connection
  ///
  /// @param[in] message The message to send through the connection
  virtual void send(boost::shared_ptr<std::string> message) = 0;

  /// @brief An overload to send a pure string
  ///
  /// @param[in] message The string that will get passed directly to the output character device.  Note that the
  /// string will be copied before it is sent out.
  virtual void send(const std::string& message);

  /// @brief Start async read loop
  virtual void receive(void) = 0;

  /// @brief Set up the connection using the provided node handle
  virtual void setup(ros::NodeHandle& nh) = 0;

  /// @brief Get this connection's name
  const std::string& getName() const;

  /// @brief Get the boost::asio service object underlying this one
  boost::asio::io_service& getIoService();

  /// @brief Get the callback
  const ReadCallback& getCallback() const;

  /// @brief Set the callback for this connection that is fired on every read
  void setCallback(const ReadCallback& _cb);

  /// @brief Disabling raw publishing is a useful optimization, but should be used
  /// with care!  This checks to make sure it's enabled
  bool getRawPublisherEnabled() const;

  void setRawPublisherEnable(bool v);

  // Make noncopyable
private:
  DsConnection(const DsConnection& other) = delete;             // non-copyconstructable
  DsConnection& operator=(const DsConnection& other) = delete;  // non-assignable

protected:
  ReadCallback callback_;
  boost::asio::io_service& io_service_;

  std::string name_;

  ros::Publisher raw_publisher_;
  ds_core_msgs::RawData raw_data_;
  bool raw_publisher_enabled_;
};
}
#endif
