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
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>
// #include <nav_msgs/GridCells.h>
// #include <geometry_msgs/Point.h>

#include <nortek_dvl_ethernet/default.h>
#include <nortek_dvl_ethernet/udp_socket_handler.h>
#include <nortek_dvl_ethernet/nortekdvl1000_structs.h>
#include <nortek_dvl_ethernet/parser.h>
#include <nortek_dvl_ethernet/NortekDF2.h>
#include <nortek_dvl_ethernet/NortekDF3.h>

/**
 * @brief This is the ROS driver class for Nortek DVL1000
 *
 * It get the bottom track, water track and current profile from the Parser through callback functions.
 * After that, individual derivated message in ROS standard message will be publish, such as velocity in twist. 
 */
class NortekDvlRos {

    //! node handler
    ros::NodeHandle nh_;

    //! private node handler
    ros::NodeHandle nh_private_;

    //! bottom track publisher
    ros::Publisher bottom_track_pub_;

    //! water track publisher
    ros::Publisher water_track_pub_;

    //! current proflie publisher
    ros::Publisher current_profile_pub_;

    //! bottom track 3-axis velocity publisher
    ros::Publisher bt_velocity_pub_;

    //! bottom track point cloud (4 points) publisher
    ros::Publisher bt_pc2_pub_;

    //! bottom track range (averaged from 4 beams) publisher
    ros::Publisher bt_range_pub_;

    //! water track 3-axis velocity publisher
    ros::Publisher wt_velocity_pub_;

    //! current profile cells publisher
    ros::Publisher cp_cells_pub_;

    //! pressure from bottom track and current profile publisher
    ros::Publisher pressure_pub_;

    //! ROS related parameters:
    //!  frame_id of published messages
    std::string frame_id_;

    //! UDP related parameters
    UdpParam udp_param_;
    
    //! Nortek related parameters: 
    //!  the angle between individual beam and DVL center line
    double beam_angle_;

    //! Nortek related parameters: 
    //!  given sound speed, correct the default sound speed used in DVL
    double sound_speed_;

    //! UDP handler object pointer
    std::shared_ptr<UDPSocketHandler> udp_handler_;

    //! Nortek parser object pointer
    std::shared_ptr<NortekDVLParser> parser_;

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
     * @param msg nortek_dvl_ethernet::NortekDF2 type ROS message
     */  
    void CallbackBT(
        const nortek_dvl_ethernet::NortekDF2& msg);

    /**
     * @brief Registered callback function to receive Nortek Water Track data
     * @param msg nortek_dvl_ethernet::NortekDF2 type ROS message
     */  
    void CallbackWT(
        const nortek_dvl_ethernet::NortekDF2& msg);

    /**
     * @brief Registered callback function to receive Nortek Current Profile data
     * @param msg nortek_dvl_ethernet::NortekDF3 type ROS message
     */  
    void CallbackCP(
        const nortek_dvl_ethernet::NortekDF3& msg);

    /**
     * @brief Convert DF21/DF22 data into velocity message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] twist_msg the 3-axis linear velocity in geometry_msgs::TwistWithCovarianceStamped 
     */
    void TrackToVelocity(
        const nortek_dvl_ethernet::NortekDF2::Ptr& track_msg, 
        geometry_msgs::TwistWithCovarianceStamped::Ptr& twist_msg);

    /**
     * @brief Convert DF21/DF22 data into pressure message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] pressure_msg the pressure in sensor_msgs::FluidPressure 
     * 
     * Nortek output the gauge pressure and unit in Bar.
     * FluidPressure need absolute pressure unit in Pascal.
     */    
    void TrackToPressure(
        const nortek_dvl_ethernet::NortekDF2::Ptr& track_msg, 
        sensor_msgs::FluidPressure::Ptr& pressure_msg);

    /**
     * @brief Convert DF21/DF22 data into point cloud message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] pc2_msg the point cloud in sensor_msgs::PointCloud2 
     * 
     * The Nortek DVL1000 only has range measurement for each beam, here,
     * we reverse the range into 3D points based on beam angle and beam location
     */    
    void TrackToPC2(
        const nortek_dvl_ethernet::NortekDF2::Ptr& track_msg, 
        sensor_msgs::PointCloud2::Ptr& pc2_msg);

    /**
     * @brief Convert DF21/DF22 data into range message
     * @param[in] track_msg the whole DF21/DF22 ROS message
     * @param[out] range_msg the point cloud in sensor_msgs::Range 
     * 
     * This is a roughly estimation, just simple averaged from 4 beam ranges
     */ 
    void TrackToRange(
        const nortek_dvl_ethernet::NortekDF2::Ptr& track_msg, 
        sensor_msgs::Range::Ptr& range_msg);

    /**
     * @brief Convert DF3 data into pressure message
     * @param[in] track_msg the whole DF3 ROS message
     * @param[out] pressure_msg the pressure in sensor_msgs::FluidPressure 
     * 
     * Nortek output the gauge pressure and unit in Bar.
     * FluidPressure need absolute pressure unit in Pascal.
     */ 
    void ProfileToPressure(
        const nortek_dvl_ethernet::NortekDF3::Ptr& profile_msg, 
        sensor_msgs::FluidPressure::Ptr& pressure_msg);

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
    //     const nortek_dvl_ethernet::NortekDF3::Ptr& profile_msg, 
    //     nav_msgs::GridCells::Ptr& cells_msg);

public:
    /**
     * @brief Default constructor
     * @param[in] nh ROS node handler
     * @param[in] nh_private ROS private node handler
     */ 
    NortekDvlRos(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private);

    /**
     * @brief Default Destructor
     */ 
    ~NortekDvlRos();    
};

#endif // NORTEK_DVL_ETHERNET_ROS_DRIVER_