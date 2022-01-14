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
#ifndef DS_CONNECTION_FACTORY_H
#define DS_CONNECTION_FACTORY_H

#include "ds_asio/ds_connection.h"
#include "ds_asio/ds_udp.h"
#include "ds_asio/ds_serial.h"
#include "ds_asio/ds_tcp_client.h"
#include "ds_asio/ds_rosraw.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <stdexcept>
#include <sstream>

namespace ds_asio
{
class DsConnectionFactory
{
public:
  /// @brief Generates a new connection object
  ///
  /// @return A boost::shared_ptr of the created connection
  ///
  /// @param[in] name Name of the connection, defines subnamespace of connection parameters
  /// @param[in] io_service   A reference to the boost::io_service object that the connection will use to perform i/o
  /// @param[in] callback     A boost::function object that will be called with the data received when the connection
  /// receives data
  /// @param[in] myNh         A reference to the nodehandle object for accessing the parameter server
  static boost::shared_ptr<DsConnection> createConnection(std::string name, boost::asio::io_service& io_service,
                                                          boost::function<void(ds_core_msgs::RawData)> callback,
                                                          ros::NodeHandle& myNh);

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  static boost::shared_ptr<DsConnection> createConnection(
      std::string name, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback,
      ros::NodeHandle& myNh, boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction)
  {
    std::string connectionType;
    myNh.getParam(ros::this_node::getName() + "/" + name + "/type", connectionType);

    if (connectionType.compare("UDP") == 0)
    {
      return boost::shared_ptr<DsUdp>(new DsUdp(io_service, name, callback, myNh));
    }
    else if (connectionType.compare("SERIAL") == 0)
    {
      return boost::shared_ptr<DsSerial>(new DsSerial(io_service, name, callback, myNh));
    }

    std::stringstream msg;
    msg << "Unable to create connection \"" << name << "\" in node " << ros::this_node::getName() << " with type \""
        << connectionType << "\"!\nLooking for type rosparam at: \""
        << ros::this_node::getName() + "/" + name + "/type\"";

    throw std::runtime_error(msg.str());
  }
};
}

#endif
