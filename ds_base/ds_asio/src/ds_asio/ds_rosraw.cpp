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

#include "ds_asio/ds_rosraw.h"

namespace ds_asio
{

DsRosRaw::DsRosRaw(boost::asio::io_service& io_service, std::string name,
    const ReadCallback& callback, ros::NodeHandle& myNh) : DsConnection(io_service, name, callback) {
  setup(myNh);
  receive();
}

void DsRosRaw::receive(void) {
  // normally used to initiate the receive chain for asio calls,
  // does nothing for this case
}

void DsRosRaw::send(boost::shared_ptr<std::string> message) {
  ds_core_msgs::RawData out;
  out.ds_header.io_time = ros::Time::now();
  out.data = std::vector<unsigned char>(message->begin(), message->end());
  out.data_direction = ds_core_msgs::RawData::DATA_OUT;

  out_pub_.publish(out);

  // we still need to respect the raw publisher convention
  if (raw_publisher_enabled_) {
    raw_publisher_.publish(out);
  }
}

void DsRosRaw::setup(ros::NodeHandle& nh) {
  DsConnection::setup(nh);

  std::string topic_in, topic_out;

  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/topic_rx", topic_in, "rosraw_rx");
  nh.param<std::string>(ros::this_node::getName() + "/" + name_ + "/topic_tx", topic_out, "rosraw_tx");

  ROS_INFO_STREAM("Subscribing to ds_core_msgs::RawData topic \"" <<topic_in <<"\" for incoming data");
  ROS_INFO_STREAM("Sending outgoing data to \"" <<topic_out <<"\"");

  raw_sub_ = nh.subscribe(topic_in, 10, &DsRosRaw::handle_receive, this);
  out_pub_ = nh.advertise<ds_core_msgs::RawData>(topic_out, 10, false);

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh.advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/" + name_ + "/raw", 1);
}

void DsRosRaw::handle_receive(ds_core_msgs::RawData msg) {

  // ignore recorded outgoing data
  if (msg.data_direction != ds_core_msgs::RawData::DATA_IN) {
    return;
  }

  // still respect the raw convention
  if (raw_publisher_enabled_) {
    raw_publisher_.publish(msg);
  }

  // fire off the callback
  if (!callback_.empty()) {
    callback_(msg);
  }

}


} // namespace ds_asio
