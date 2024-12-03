/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/


#include <nortek_dvl_ethernet/udp_socket_handler.h>
#include <nortek_dvl_ethernet/ros_driver.h>


NortekDvlRos::NortekDvlRos(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load parameters
    LoadParam();

    // setup ros
    SetupRos();

    // init the UDP interface
    InitDataInterface();
}

NortekDvlRos::~NortekDvlRos() {

    //! TODO: close something
}

void NortekDvlRos::LoadParam()
{
    // UDP configuration
    nh_private_.param<int>("UDP/udp_rx", udp_param_.udp_rx, DEFAULT_UDP_RX);    
    nh_private_.param<int>("UDP/udp_tx", udp_param_.udp_tx, DEFAULT_UDP_TX);    
    nh_private_.param<std::string>("UDP/udp_address", udp_param_.udp_address, DEFAULT_UDP_ADDRESS);
    nh_private_.param<int>("UDP/buffer_size", udp_param_.buffer_size, DEFAULT_UDP_BUFFER);    
}

void NortekDvlRos::SetupRos()
{
    df21_pub_ = nh_.advertise<nortek_dvl_ethernet::NortekDF21>("DF21", 10);

    df3_pub_ = nh_.advertise<nortek_dvl_ethernet::NortekDF3>("DF3", 10);

    test_sub_ = nh_.subscribe("test_data_sub", 10, &NortekDvlRos::CallbackTest, this);
}

void NortekDvlRos::InitDataInterface()
{
    // Set up callback for received data from the socket
    udp_handler_ = std::make_shared<UDPSocketHandler>(udp_param_);

    udp_handler_->SetCallback(
        std::bind(&NortekDvlRos::CallbackUDP, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Set up callback for parsed data (bottom track, current profile, water track)
    parser_ = std::make_shared<NortekDVLParser>();  

    parser_->SetDF21Callback(
        std::bind(&NortekDvlRos::CallbackDF21, this, std::placeholders::_1)
    );

    parser_->SetDF3Callback(
        std::bind(&NortekDvlRos::CallbackDF3, this, std::placeholders::_1)
    );    
}

void NortekDvlRos::CallbackDF21(const nortek_dvl_ethernet::NortekDF21& df21)
{
    nortek_dvl_ethernet::NortekDF21 df21_msg = df21;
    //! TODO: add param
    df21_msg.header.frame_id = "nortek_dvl";
    df21_pub_.publish(df21_msg);
}

void NortekDvlRos::CallbackDF3(const nortek_dvl_ethernet::NortekDF3& df3)
{
    nortek_dvl_ethernet::NortekDF3 df3_msg = df3;
    //! TODO: add param
    df3_msg.header.frame_id = "nortek_dvl";
    df3_pub_.publish(df3_msg);
}


void NortekDvlRos::CallbackUDP(const uint8_t* data, std::size_t size) {

    auto io_time = ros::Time::now().toSec();

    auto result = parser_->Parse(data, size, io_time);

    if(result == nortek_dvl_structs::parserID::ERROR)
    {
        ROS_ERROR("parsed something wroing");
    }

}

void NortekDvlRos::CallbackTest(const std_msgs::Float32::ConstPtr &msg) {
    //! DEBUG:
    ROS_INFO("%s: Recv: %f", ros::this_node::getName().c_str(), msg->data);
}