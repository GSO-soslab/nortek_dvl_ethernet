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
// Created by ivaughn on 1/19/18.
//

#ifndef PROJECT_DS_MOCK_CONNECTION_H
#define PROJECT_DS_MOCK_CONNECTION_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <deque>

namespace ds_asio
{
/// \brief This is a mock class intended to let test code set up
class DsMockConnection : public DsConnection
{
public:
  DsMockConnection(boost::asio::io_service& io_service);
  virtual ~DsMockConnection();

  // public interface
  virtual void receive(void);
  virtual void send(boost::shared_ptr<std::string> message);

  /// \brief run the test.  Will return when the test is complete.
  void run();

  /// \brief Get the vector of stuff written (used AFTER) the test
  /// has been run
  const std::deque<std::string>& Written() const;

  /// \brief Get the vector of stuff to Read (used BEFORE the test!)
  const std::deque<ds_core_msgs::RawData>& ToRead() const;
  std::deque<ds_core_msgs::RawData>& ToRead();

  boost::asio::io_service& getIoService();

  /// \brief Flag that a write has already happened, and the mock connection
  /// should start by immediately emitting its first reply
  void setWriteDuringStartup();

  /// @brief Required for base class, performs no action.
  void setup(ros::NodeHandle& nh) override;

protected:
  /// \brief A vector of strings written by the thing using this to test
  std::deque<std::string> written;

  /// \brief A vector of strings to be ready by the thing under test
  std::deque<ds_core_msgs::RawData> toRead;

  void sendNextMessage();

  bool initializing;
  bool writeDuringStartup;

private:
  // make noncopyable
  DsMockConnection(const DsMockConnection& other) = delete;
  DsMockConnection& operator=(const DsMockConnection& other) = delete;
};
}

#endif  // PROJECT_DS_MOCK_CONNECTION_H
