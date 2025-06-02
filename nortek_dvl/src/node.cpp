/*
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

    Authors: Lin Zhao <linzhao@uri.edu>
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

// #include "ros/ros.h"
// #include <nortek_dvl_ethernet/udp_socket_handler.h>
// #include <nortek_dvl_ethernet/ros_driver.h>

// int main(int argc, char** argv) {

//     ros::init(argc, argv,"Nortek_DVL1000_ROS_driver");

//     ros::NodeHandle nh("");
//     ros::NodeHandle nh_private("~");

//     NortekDvlRos node(nh, nh_private);

//     // Use ROS spinner to handle callbacks
//     ros::AsyncSpinner spinner(2);
//     spinner.start();

//     // Keep the main thread alive
//     ros::waitForShutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <nortek_dvl/ros_driver.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NortekDvlRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

