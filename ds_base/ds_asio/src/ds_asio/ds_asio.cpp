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
#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_callbackqueue.h"

namespace ds_asio
{
boost::shared_ptr<DsConnection> DsAsio::addConnection(std::string name, const ReadCallback& callback,
                                                      ros::NodeHandle& myNh)
{
  if (connections.find(name) != connections.end())
  {
    ROS_ERROR_STREAM("Unable to add connection: " << name << ".  Connection by that name already exists.");
    return {};
  }

  auto connection = DsConnectionFactory::createConnection(name, io_service, callback, myNh);
  ROS_INFO_STREAM("Created new connection named: " << name);
  connections.insert({ name, connection });
  return connection;
}

boost::shared_ptr<IoSM> DsAsio::addIoSM(std::string iosm_name, std::string conn_name, const IoCommand::ReadCallback& callback,
                                        ros::NodeHandle& myNh)
{
  boost::shared_ptr<DsConnection> conn = addConnection(conn_name, ReadCallback(), myNh);
  boost::shared_ptr<IoSM> ret(new IoSM(io_service, iosm_name, callback));
  // setConnection will also setup our connection to the callback function
  ret->setConnection(conn);

  return ret;
}

boost::shared_ptr<DsConnection> DsAsio::connection(const std::string& name)
{
  try
  {
    return connections.at(name);
  }
  catch (std::out_of_range& e)
  {
    ROS_ERROR_STREAM("No connection named: " << name);
    return {};
  }
}

std::map<std::string, boost::shared_ptr<DsConnection> >
DsAsio::startConnections(ros::NodeHandle& myNh, std::map<std::string, ReadCallback> mapping)
{
  std::map<std::string, boost::shared_ptr<DsConnection> > handle;

  // Iterate over mapping keys
  for (const auto& myPair : mapping)
  {
    ROS_INFO_STREAM("Looking for connection: " << myPair.first);
    handle[myPair.first] = this->addConnection(myPair.first, mapping[myPair.first], myNh);
  }

  return handle;
}

DsAsio* DsAsio::asio(void)
{
  return this;
}

void DsAsio::signalHandler(const boost::system::error_code& error, int signal_number)
{
  if (!error)
  {
    ROS_INFO_STREAM("A signal occurred, shutting down ROS and exiting...");
    ros::shutdown();
    exit(0);
  }
  else
  {
    ROS_INFO_STREAM("An error in the signal handler occurred");
  }
}

DsCallbackQueue* DsAsio::dsCallbackQueue()
{
  return callback_queue_.get();
}

ros::CallbackQueueInterface* DsAsio::callbackQueue()
{
  return static_cast<ros::CallbackQueueInterface*>(dsCallbackQueue());
}

DsAsio::DsAsio() : callback_queue_(std::unique_ptr<DsCallbackQueue>(new DsCallbackQueue(&io_service)))
{
}

DsAsio::DsAsio(int argc, char** argv, const std::string& name) : DsAsio()
{
  ros::init(argc, argv, name);

  ROS_INFO_STREAM(ros::this_node::getName());
  ROS_INFO_STREAM(ros::this_node::getNamespace());
}

DsAsio::~DsAsio() = default;

void DsAsio::run(void)
{
  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  boost::asio::signal_set signals(io_service, SIGINT);
  signals.async_wait(boost::bind(&DsAsio::signalHandler, this, _1, _2));
  io_service.run();
}
}
