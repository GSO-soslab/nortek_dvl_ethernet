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

#ifndef NORTEK_DVL_ETHERNET_ROS_DRIVER_
#define NORTEK_DVL_ETHERNET_ROS_DRIVER_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nortek_msgs/msg/nortek_df2.hpp>
#include <nortek_msgs/msg/nortek_df3.hpp>
// #include <nav_msgs/msg/grid_cells.hpp>
// #include <geometry_msgs/msg/point.hpp>

#include <nortek_dvl/default.h>
#include <nortek_dvl/udp_socket_handler.h>
#include <nortek_dvl/nortekdvl1000_structs.h>
#include <nortek_dvl/parser.h>

/**
 * @brief This is the ROS driver class for Nortek DVL1000
 *
 * It get the bottom track, water track and current profile from the Parser through callback functions.
 * After that, individual derivated message in ROS standard message will be publish, such as velocity in twist. 
 */
class NortekDvlRos : public rclcpp::Node
{
public:
    NortekDvlRos();

private:
    // ===================================================================== //
    // ROS variables
    // ===================================================================== // 

    //! bottom track publisher
    rclcpp::Publisher<nortek_msgs::msg::NortekDF2>::SharedPtr 
        bottom_track_pub_;

    //! water track publisher
    rclcpp::Publisher<nortek_msgs::msg::NortekDF2>::SharedPtr 
        water_track_pub_;   
        
    //! current proflie publisher
    rclcpp::Publisher<nortek_msgs::msg::NortekDF3>::SharedPtr 
        current_profile_pub_;   

    //! bottom track 3-axis velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr 
        bt_velocity_pub_;   

    //! bottom track point cloud (4 points) publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 
        bt_pc2_pub_;  

    //! bottom track range (averaged from 4 beams) publisher
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr 
        bt_range_pub_;  

    //! water track 3-axis velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr 
        wt_velocity_pub_;  

    //! current profile cells publisher
    // rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr 
    //     cp_cells_pub_;     

    //! pressure from bottom track and current profile publisher
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr 
        pressure_pub_; 

    //! depth odometry from bottom track and current profile publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
        depth_pub_; 

    // ===================================================================== //
    // ROS related parameters:
    // ===================================================================== //

    //!  frame_id of published messages
    std::string sensor_frame_id_;

    //!  frame_id of pressure depth odometry
    std::string world_frame_id_;    

    //! fluid_density used for depth calulcation from pressure
    double fluid_density_;

    //! covariance for depth estimnation
    double depth_cov_;

    //! UDP related parameters
    UdpParam udp_param_;
     
    //!  the angle between individual beam and DVL center line
    double beam_angle_;

    // ===================================================================== //
    // Nortek related parameters: 
    // ===================================================================== //       

    //!  given sound speed, correct the default sound speed used in DVL
    double sound_speed_;

    //! UDP handler object pointer
    std::shared_ptr<UDPSocketHandler> udp_handler_;

    //! Nortek parser object pointer
    std::shared_ptr<NortekDVLParser> parser_;        

    // ===================================================================== //
    // Functions
    // ===================================================================== //     

    /**
     * @brief Load ROS parameters
     */
    void LoadParam();

    /**
     * @brief Setup ROS 
     *
     * Such as setup publisher, subscriber...
     */
    void SetupRos();
    
    /**
     * @brief Initialize the data interface
     *
     * Setup the UDP handler to receive the raw binary data.
     * Setup the Nortek parser to receive BT,WT,CP data.
     */    
    void InitDataInterface();

    /**
     * @brief Registered callback function to receive UDP raw data
     */  
    void CallbackUDP(
        const uint8_t* data, 
        std::size_t size);

    /**
     * @brief Registered callback function to receive Nortek Bottom Track data
     * @param msg nortek_msgs::msg::NortekDF2 type ROS message
     */  
    void CallbackBT(
        const nortek_msgs::msg::NortekDF2& msg);

    /**
     * @brief Registered callback function to receive Nortek Water Track data
     * @param msg nortek_msgs::msg::NortekDF2 type ROS message
     */  
    void CallbackWT(
        const nortek_msgs::msg::NortekDF2& msg);

    /**
     * @brief Registered callback function to receive Nortek Current Profile data
     * @param msg nortek_msgs::msg::NortekDF3 type ROS message
     */  
    void CallbackCP(
        const nortek_msgs::msg::NortekDF3& msg);

    /**
     * @brief Convert DF21/DF22 data into velocity message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] twist_msg the 3-axis linear velocity in geometry_msgs::TwistWithCovarianceStamped 
     */
    void TrackToVelocity(
        const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
        geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr& twist_msg);

    /**
     * @brief Convert DF21/DF22 data into pressure message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] pressure_msg the pressure in sensor_msgs::FluidPressure 
     * 
     * Nortek output the gauge pressure and unit in Bar.
     * FluidPressure need absolute pressure unit in Pascal.
     */    
    void TrackToPressure(
        const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
        sensor_msgs::msg::FluidPressure::SharedPtr& pressure_msg);

    /**
     * @brief Convert DF21/DF22 data into depth odometry message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] depth_odom_msg the odometry message only using depth
     * 
     * Convert the gauge pressure to depth and construct it as odometry message
     */    
    void TrackToDepthOdom(
        const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
        nav_msgs::msg::Odometry::SharedPtr& depth_odom_msg);

    /**
     * @brief Convert DF21/DF22 data into point cloud message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] pc2_msg the point cloud in sensor_msgs::PointCloud2 
     * 
     * The Nortek DVL1000 only has range measurement for each beam, here,
     * we reverse the range into 3D points based on beam angle and beam location
     */    
    void TrackToPC2(
        const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
        sensor_msgs::msg::PointCloud2::SharedPtr& pc2_msg);

    /**
     * @brief Convert DF21/DF22 data into range message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] range_msg the point cloud in sensor_msgs::Range 
     * 
     * This is a roughly estimation, just simple averaged from 4 beam ranges
     */ 
    void TrackToRange(
        const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
        sensor_msgs::msg::Range::SharedPtr& range_msg);

    /**
     * @brief Convert DF3 data into pressure message
     * @param[in] track_msg the whole DF3 ROS message
     * @param[out] pressure_msg the pressure in sensor_msgs::FluidPressure 
     * 
     * Nortek output the gauge pressure and unit in Bar.
     * FluidPressure need absolute pressure unit in Pascal.
     */ 
    void ProfileToPressure(
        const nortek_msgs::msg::NortekDF3::ConstSharedPtr& profile_msg, 
        sensor_msgs::msg::FluidPressure::SharedPtr& pressure_msg);

    /**
     * @brief Convert DF3 data into depth odometry message
     * @param[in] profile_msg the whole DF3 ROS message
     * @param[out] depth_odom_msg the odometry message only using depth
     * 
     * Convert the gauge pressure to depth and construct it as odometry message
     */    
    void ProfileToDepthOdom(
        const nortek_msgs::msg::NortekDF3::ConstSharedPtr& profile_msg, 
        nav_msgs::msg::Odometry::SharedPtr& depth_odom_msg);

    /**
     * @brief Convert DF3 data into cells message
     * @param[in] track_msg the whole DF3 ROS message
     * @param[out] cells_msg the cells in nav_msgs::GridCells
     * 
     * cell_width: beam size
     * cell_height: cell size
     * 
     * Each c_i has three measuremet:
     * x=velocity, y=amplitude,z=correlation
     * 
     *                      Beams Direction
     *                   beam1 beam2 beam3 beam4
     *                  |----------------------->
     * Depth      cell1 |c1    c2    c3    c4   
     * Direction  cell2 |c5    c6    c7    c8
     *            cell3 |c9    c10   c11   c12
     *                 \ /
     */ 
    // void ProfileToCells(
    //     const nortek_msgs::msg::NortekDF3::ConstSharedPtr& profile_msg, 
    //     nav_msgs::msg::GridCells::SharedPtr& cells_msg);    
};

#endif // NORTEK_DVL_ETHERNET_ROS_DRIVER_
