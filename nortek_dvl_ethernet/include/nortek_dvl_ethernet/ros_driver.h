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

#ifndef NORTEK_DVL_ETHERNET_ROS_DRIVER_
#define NORTEK_DVL_ETHERNET_ROS_DRIVER_

// ros
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
// customized
#include <nortek_dvl_ethernet/default.h>
#include <nortek_dvl_ethernet/udp_socket_handler.h>
#include <nortek_dvl_ethernet/nortekdvl1000_structs.h>
#include <nortek_dvl_ethernet/parser.h>
#include <nortek_dvl_ethernet/NortekDF21.h>
#include <nortek_dvl_ethernet/NortekDF3.h>

class NortekDvlRos {
    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher df21_pub_;

    ros::Publisher df3_pub_;

    ros::Subscriber test_sub_;

    std::shared_ptr<UDPSocketHandler> udp_handler_;

    std::shared_ptr<NortekDVLParser> parser_;

    UdpParam udp_param_;

    void LoadParam();

    void SetupRos();
    
    void InitDataInterface();

    void CallbackTest(const std_msgs::Float32::ConstPtr &msg);

    void CallbackUDP(const uint8_t* data, std::size_t size);

    void CallbackDF21(const nortek_dvl_ethernet::NortekDF21& df21);

    void CallbackDF3(const nortek_dvl_ethernet::NortekDF3& df3);
    
public:
    NortekDvlRos(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private);

    ~NortekDvlRos();    
};

#endif // NORTEK_DVL_ETHERNET_ROS_DRIVER_