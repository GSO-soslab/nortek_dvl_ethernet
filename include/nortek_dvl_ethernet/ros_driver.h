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

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>

#include <nortek_dvl_ethernet/default.h>
#include <nortek_dvl_ethernet/udp_socket_handler.h>
#include <nortek_dvl_ethernet/nortekdvl1000_structs.h>
#include <nortek_dvl_ethernet/parser.h>
#include <nortek_dvl_ethernet/NortekDF2.h>
#include <nortek_dvl_ethernet/NortekDF3.h>

class NortekDvlRos {
    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher bottom_track_pub_;

    ros::Publisher water_track_pub_;

    ros::Publisher current_profile_pub_;

    ros::Publisher bt_velocity_pub_;

    ros::Publisher bt_pc2_pub_;

    ros::Publisher bt_range_pub_;

    ros::Publisher wt_velocity_pub_;

    ros::Publisher pressure_pub_;

    ros::Subscriber test_sub_;

    std::shared_ptr<UDPSocketHandler> udp_handler_;

    std::shared_ptr<NortekDVLParser> parser_;

    UdpParam udp_param_;
    
    double beam_angle_;

    double sound_speed_;

    void LoadParam();

    void SetupRos();
    
    void InitDataInterface();

    void CallbackTest(const std_msgs::Float32::ConstPtr &msg);

    void CallbackUDP(const uint8_t* data, std::size_t size);

    void CallbackBT(const nortek_dvl_ethernet::NortekDF2& msg);

    void CallbackWT(const nortek_dvl_ethernet::NortekDF2& msg);

    void CallbackCP(const nortek_dvl_ethernet::NortekDF3& msg);

    void TrackToVelocity(
        const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
        geometry_msgs::TwistWithCovarianceStamped::Ptr& twist_msg);
    
    // BT measureme the gauge pressure and unit in Bar
    // FluidPressure need absolute pressure unit in Pascal
    void TrackToPressure(
        const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
        sensor_msgs::FluidPressure::Ptr& pressure_msg);

    void TrackToPC2(
        const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
        sensor_msgs::PointCloud2::Ptr& pc2_msg);

    // the averaged range from 4 beams
    void TrackToRange(
        const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
        sensor_msgs::Range::Ptr& range_msg);

public:
    NortekDvlRos(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private);

    ~NortekDvlRos();    
};

#endif // NORTEK_DVL_ETHERNET_ROS_DRIVER_